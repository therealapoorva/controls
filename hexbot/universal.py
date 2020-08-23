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
		ml_torus_identification_model(cv_image)
		
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
	if cv_to_detect_color == ml_tag_label():
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
