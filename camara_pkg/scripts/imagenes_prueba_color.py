#!/usr/bin/env python

import rospy
from custom_msgs.srv import ColorDeImagen
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
args = vars(ap.parse_args())

bridge = CvBridge()

def enviar_img(img):
	rospy.wait_for_service("/get_color")
	try:
		service = rospy.ServiceProxy("/get_color", ColorDeImagen)
		color = service(img,200,200,10,10)
		rospy.loginfo(color)
	except rospyServicesException as e:
		rospy.logerr(e)

if __name__ == '__main__':

	
	rospy.init_node('imagenes_prueba')
	
	img = cv2.imread('/home/fitocuan/catkin_ws/src/camara_pkg/scripts/crop-img/'+args["image"])
	#print(img.shape)
	img = bridge.cv2_to_imgmsg(img, encoding = "bgr8")
	enviar_img(img)
	
