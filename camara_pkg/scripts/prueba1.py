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



def adaptive_thresholding(rgb_image, threshol_value):
	adaptive_thresholding_image = cv2.adaptiveThreshold(rgb_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 																cv2.THRESH_BINARY, threshol_value,2)
	cv2.imshow("Adaptive Threshold Image", adaptive_thresholding_image)
	cv2.waitKey(3)

def track(rgb_image):

	hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
	redLower = (30,50,100)
	redUpper = (40,255,255)
	rgb_image = cv2.GaussianBlur(hsv,(7,7),0)
	binary_image = cv2.inRange(rgb_image, redLower, redUpper)
	
	_, contours, hierachy = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	
	black_image = np.zeros([binary_image.shape[0],binary_image.shape[1],3 ], 'uint8')
	
	for c in contours:
		area = cv2.contourArea(c)
		perimeter = cv2.arcLength(c,True)
		((x,y),radius) = cv2.minEnclosingCircle(c)
		if(area>500):
			cv2.drawContours(rgb_image,[c], -1, (150,250,150),1)
			cv2.drawContours(binary_image,[c], -1, (150,250,150),1)
			cx,cy = get_contour_center(c)
			cv2.circle(rgb_image,(cx,cy),(int)(radius),(0,0,255),1)
			cv2.circle(black_image,(cx,cy),(int)(radius),(0,0,255),1)
			cv2.circle(black_image,(cx,cy),5,(150,150,255),-1)
			print("Area: {}, Perimeter: {}".format(area,perimeter))
			msg = Point32()
			msg.x = cx
			msg.y = cy
			msg.z = 0
			pub.publish(msg)
		
	print("Number of Contours: {}".format(len(contours)))
	cv2.imshow("RGB Image", rgb_image)
	cv2.imshow("Black Image", black_image)
	cv2.imshow("Binary Image", binary_image)
	
def get_contour_center(contour):
	M = cv2.moments(contour)
	cx = -1
	cy = -1
	
	if(M['m00']!=0):
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
	
		
	return cx,cy
			
				

def image_callback(ros_image):
	print("got image")
	global bridge
	
	try:
		cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
	except CvBridgeError as e:
		print(e)
		
	threshol_value = 115
	
	cv2.imshow("Image", cv_image)
	cv2.waitKey(3)
	
	#adaptive_thresholding(cv_image, threshol_value)
	track(cv_image)
	
	
	
	
	


if __name__ == '__main__':
	print("a")
	
	rospy.init_node('image_converter', anonymous = True)
	image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
	pub = rospy.Publisher("/ball_coords", Point32, queue_size = 10)
	
	rate = rospy.Rate(2)
	
	
	rate.sleep()
		
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("shutting down")
	cv2.destroyAllWindows()

	
	




	
		
		


