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


def image_callback(ros_image):
	print("got image2")
	global bridge
	
	try:
		cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	except CvBridgeError as e:
		print(e)
			
	cv2.imshow("Image2", cv_image)

	cv2.waitKey(3)
	

if __name__ == '__main__':
	print("a")

	
	rospy.init_node('image_converter', anonymous = True)
	image_sub1 = rospy.Subscriber("/cameras/usb_cam2/image_raw", Image, image_callback)
	#image_sub2 = rospy.Subscriber("/cameras/usb_cam2/image_raw", Image, image_callback2)

	
	rate = rospy.Rate(3)	
	
	rate.sleep()
		
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()

