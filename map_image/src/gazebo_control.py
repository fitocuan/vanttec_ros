#!/usr/bin/env python

from std_msgs.msg import String
from std_msgs.msg import Duration
from std_msgs.msg import Time



from custom_msgs.srv import ColorDeImagen
from custom_msgs.srv import DistanceCal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList

import subprocess
import os

import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import imutils
import argparse
import numpy as np
import time
import rospy
import cv2
import math

from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3



class Gazebo_Model:
    def __init__(self):
        pass

    def move(self,l,r):
        """ Calculates distance using get_distance service """
        point_left, point_right = Point(),Point()
        point_left.x = 0
        point_left.y = 0
        point_left.z = 0
        point_right.x = 0
        point_right.y = 0
        point_right.z = 0

        wrench_left,wrench_right = Wrench(),Wrench()
        force_left,force_right = Vector3(),Vector3()
        torque_left,torque_right = Vector3(),Vector3()

        force_left.x = 0
        force_left.y = 0
        force_left.z = 0
        torque_left.x = 0
        torque_left.y = l
        torque_left.z = 0

        force_right.x = 0
        force_right.y = 0
        force_right.z = 0
        torque_right.x = 0
        torque_right.y = r
        torque_right.z = 0

        wrench_left.force = force_left
        wrench_left.torque = torque_left

        wrench_right.force = force_right
        wrench_right.torque = torque_right

        rospy.wait_for_service("/gazebo/apply_body_wrench")

        service = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

        resp = service("back_left_wheel",'',point_left,wrench_left,rospy.Time(),rospy.Duration(0.5,0))
        resp = service("back_right_wheel",'',point_right,wrench_right,rospy.Time(),rospy.Duration(0.5,0))


   
        

if __name__ == '__main__':
    try:
        rospy.init_node('obj_list_fusion')

        rate = rospy.Rate(10) # 10Hz

        D = Gazebo_Model()

        D.move(3,3)
        D.move(-3,-3)

    except rospy.ROSInterruptException:
        pass
