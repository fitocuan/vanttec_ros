#!/usr/bin/env python


import rospy
import cv2
from custom_msgs.srv import ColorDeImagen
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

def callback_color(img):
	global bridge
	
	image = img.imagen
	x = img.x 
	y = img.y
	h = img.h
	w = img.w
	image = bridge.imgmsg_to_cv2(image, "bgr8")
	#image = image[y:y+h,x:x+w]

	
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	
	colors = []

	green_up = np.array([255,220,220])
	green_low = np.array([30,165,78])
	green_mask = cv2.inRange(hsv,green_low,green_up)
	green = np.sum(green_mask)
	colors.append(green)

	red_up = np.array([61,252,255])
	red_low = np.array([0,137,0])
	red_mask = cv2.inRange(hsv,red_low,red_up)
	red = np.sum(red_mask)
	colors.append(red)

	blue_up = np.array([125,230,255])
	blue_low = np.array([97,76,52])
	blue_mask = cv2.inRange(hsv,blue_low,blue_up)
	blue = np.sum(blue_mask)
	colors.append(blue)

	colors_name = ["green", "red", "blue"]

	hist = cv2.calcHist([hsv], [0], None, [6], [0, 180])
	
	colors = ["red", "yellow-green", "green", "blue", "blue-magenta", "magenta-red"]
	hist_list = [hist[i][0] for i in range(len(hist))]
	hist_dic = dict(zip(hist_list, colors))
	
	ret = str(hist_dic[max(hist_dic)])
	ret = colors_name[colors.index(max(colors))]
	return ret
	


if __name__ == '__main__':

	rospy.init_node('color_detector_server')
	rospy.loginfo("Node created!")

	service = rospy.Service("/get_color", ColorDeImagen, callback_color)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		rate.sleep()
