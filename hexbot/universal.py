#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


left=20
front=20
command="start"

def go_left(value):
	global command
	msg = Twist()
	state_description = 'going_left'
	if command=='left':
		if(value>0.75):
			rospy.loginfo(state_description)
			msg.linear.y = 8.0
			velocity_pub.publish(msg)
		else:
			msg.linear.y=0
			velocity_pub.publish(msg)
			command='front'

def go_front(value):
	global command
	msg = Twist()
	state_description = "going_forward" 
	if command=='front':
		if(value>1):
			rospy.loginfo(state_description)
			msg.linear.x = 10
			velocity_pub.publish(msg)
		else:
			msg.linear.x=0
			velocity_pub.publish(msg)
			command='stop'

def callback_frontlaser(msg):
	global command
	global front
	global left
	
	left = min(msg.ranges[710:])
	front = min(msg.ranges[355:365])
	if command=='start':
		command='left'
	if command=='left':
		go_left(left)
	if command=='front':
		go_front(front)
	if command=='stop':
		command='start_scanning_objects'

def centrefinding(cv_image):
	imfinal = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	ret,thresh = cv2.threshold(imfinal,230,255,cv2.THRESH_BINARY)
	image, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	if(len(contours)>1):
		cv_image = cv2.drawContours(cv_image, contours, 1, (0,255,0), 3)
		M = cv2.moments(contours[1])
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])

		cv_image = cv2.circle(cv_image, (cx,cy), radius=5, color=(255,0,0), thickness=-1)
		cv_image = cv2.circle(cv_image, (cv_image.shape[1]//2,cv_image.shape[0]//2), radius=10, color=(0,0,255), thickness=-1)
		a = cv_image.shape[1]//2
		b = cx
		if abs(a-b)<5:
			v=Twist()
			v.linear.y=0
			velocity_pub.publish(v)
		else:
			v=Twist()
			v.linear.y= -0.5
			velocity_pub.publish(v)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)
	else:
		v=Twist()
		v.linear.y= -0.5
		velocity_pub.publish(v)

		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)


def load(img_name):
	img=cv2.imread(img_name,1)
	img=cv2.resize(img,(960,540))
	img_new=img[106:488,245:780]
	#img_new=cv2.resize(img_new,(b,h))
	#img_new=img_new.reshape(h,b,1)
	return img_new
	#if using solid work images and want to crop out some stuff
def cv_to_detect_shape_color():
	img=load('cyan.png')
	#Otherwise
	#img=cv2.imread('b.png')
	img_rgb=img.copy()
	img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	_,thresh = cv2.threshold(gray,200,255,cv2.THRESH_BINARY_INV)
	_, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,
		cv2.CHAIN_APPROX_SIMPLE)
	#print(len(contours))
	area=0
	for cnts in contours:
		if cv2.contourArea(cnts)>area:
			cnt=cnts
			area=cv2.contourArea(cnts)
	peri=cv2.arcLength(cnt,True)
	epsilon=0.01*peri
	approx=cv2.approxPolyDP(cnt,epsilon,True)
	img=cv2.drawContours(img,[cnt],0,(0,255,0),3)
	x,y,w,h=cv2.boundingRect(cnt)
	_,(wr,hr),_=cv2.minAreaRect(cnt)
	ellipse=cv2.fitEllipse(cnt)
	img = cv2.ellipse(img,ellipse,(0,0,255),2)
	a=ellipse[1][0]
	b=ellipse[1][1]
	if len(approx)<=6 :
		shape="cuboid"
	elif 0.95<w*1.0/h<1.05 :
		shape="sphere"
	elif wr*hr>w*h+0.1*area :
		shape="cone"
	elif w*1.0/h<1.5 :
		shape="it's tall not wide"
	elif abs(0.786*a*b-area)<0.05*area:
		shape="torus"
	else :
		shape="I don't know"
	mask=np.zeros_like(gray)
	mask=cv2.drawContours(mask,[cnt],0,255,-1)
	cv2.imshow('mask',mask)
	mask=mask/255.0
	num=np.sum(mask)
	r=np.sum(img_rgb[:,:,2]*mask)/num
	g=np.sum(img_rgb[:,:,1]*mask)/num
	b=np.sum(img_rgb[:,:,0]*mask)/num
	color="unidentified"
	if (r-g)**2+(g-b)**2+(b-r)**2<100 :
		bright=np.sum(gray*mask)/num
		if bright<10 :
			color="black"
		else :
			color="gray"
	else :
		img_hsv=img_hsv*1.0
		hue=np.sum(mask*img_hsv[:,:,0])/np.sum(mask)
		if hue<=5 or hue>=175 :
			color="red"
		else :
			colors=["green","navy_blue","cyan","yellow","brown"]
			hue_values=[60,120,90,30,15]
			for i in range(5) :
				if hue_values[i]-5<=hue<=hue_values[i]+5 :
					color=colors[i]
	return shape, color
	cv2.imshow('frame',img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def callback_opencv(data):
	global command
	if command=='start_scanning_objects':
		command='start_opencv'
	if command=='start_opencv':
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		centrefinding(cv_image)
'''	elif command=='torus_search':
		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		cv_to_detect_shape_color()[0]
		
def pickup():
	global command
	if ml_torus_identification_model() == 1:
		arm_pickup_mechanism_and_put_on_bot_body()
	else:
		command='torus_search'
		callback_opencv(data)
def see_tag_and_put():
	global command
	pick_up_from_bot()
	cv_to_detect_color()
	if cv_to_detect_shape_color()[1] == ml_tag_label():
		put_in_rod()
	else:
		go_front(value)
		continue
		'''

def main():
	global command
	global left
	command='start'
	global velocity_pub
	global sub_laser_left
	rospy.init_node('main')
	velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	sub_laser_front = rospy.Subscriber('/hexbot/laser/scan', LaserScan , callback_frontlaser)
	image_sub = rospy.Subscriber("/hexbot/camera1/image_raw",Image,callback_opencv)
	#image_sub2 = rospy.Subscriber("/hexbot/camera1/image_raw",Image, pickup)
	#image_sub3 = rospy.Subscriber("/hexbot/camera1/image_raw",Image, see_tag_and_put)
	
	rospy.spin()

if __name__ == '__main__':
	main()
