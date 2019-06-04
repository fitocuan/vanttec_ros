#!/usr/bin/env python 


import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()
img2 = Image()
img1 = Image()


def modImg():
	global img1
	global img2
	
	cv2.putText(img2,'Camara 123',(10,50),cv2.FONT_HERSHEY_SIMPLEX ,1, (255,255,255), 2)
	cv2.putText(img1,'Camara 456',(10,50),cv2.FONT_HERSHEY_SIMPLEX ,1, (255,255,255), 2)
	
	
def image_callback2(ros_image):
	global img2
	global bridge

	try:
		print("got image2")

		
		img2 = bridge.imgmsg_to_cv2(ros_image, "bgr8")
		#modImg()
		cv2.putText(img2,'Camara 1',(10,50),cv2.FONT_HERSHEY_SIMPLEX ,1, (255,255,255), 2)
		
		img2 = bridge.cv2_to_imgmsg(img2, encoding = "bgr8")
		

		msg = img2
		
		pub2.publish(msg)
		
	except CvBridgeError as e:
		print(e)
			




def image_callback(ros_image):
	global img1
	global bridge

	try:
		print("got image1")
		img1 = bridge.imgmsg_to_cv2(ros_image, "bgr8")
		

		cv2.putText(img1,'Camara 2',(10,50),cv2.FONT_HERSHEY_SIMPLEX ,1, (0,0,0), 2)
		
		img1 = bridge.cv2_to_imgmsg(img1, encoding = "bgr8")
		
		msg = img1
		
		pub.publish(msg)

	except CvBridgeError as e:
		print(e)
			

	

if __name__ == '__main__':
	print("a")
	
	
	rospy.init_node('image_converter', anonymous = True)
	image_sub1 = rospy.Subscriber("/usb_cam1/image_raw", Image, image_callback)
	image_sub2 = rospy.Subscriber("/usb_cam2/image_raw", Image, image_callback2)
	
	
	
	pub = rospy.Publisher("/imagen_modificada", Image, queue_size = 10)
	pub2 = rospy.Publisher("/imagen_modificada2", Image, queue_size = 10)
	

			
	rate = rospy.Rate(5)	
	
	rate.sleep()
		
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()
