#!/usr/bin/env python

from std_msgs.msg import String

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



class obj_mapping:
    def __init__(self):
        

        self.fusion_objs_det = ObjDetectedList()
        self.zed_obj_list = ObjDetectedList()

        rospy.Subscriber("/lidar_objs_det", ObjDetectedList, self.callback_lidar_cp)
        rospy.Subscriber("/zed_objs_det", ObjDetectedList, self.callback_zed_cp)

        self.obj_pub = rospy.Publisher('/fusion_objs_det', ObjDetectedList, queue_size=10)
        
        

    def spawn_model(self, name, model, x, y , z):
        """ Calculates distance using get_distance service """
        cmd = "rosrun gazebo_ros spawn_model"
        subprocess.Popen(cmd + " -database " + name + " -gazebo -model " + model+ " -y " + str(y) + " -x " + str(x) , shell = True)

    def callback_lidar_cp(self,msg):
        print("lidar")

        diff = 0.2
        self.fusion_objs_det = msg

        for zed_item in self.zed_obj_list.objects:
            for lidar_item in self.fusion_objs_det.objects:
                if(abs(zed_item.X - lidar_item.X) < diff and abs(zed_item.Y - lidar_item.Y) < diff):
                    lidar_item.color = zed_item.color
                    lidar_item.clase = zed_item.clase

        self.obj_pub.publish(self.fusion_objs_det)


    def callback_zed_cp(self,msg):
        print("zed")
        self.zed_obj_list = msg
   
        

if __name__ == '__main__':
    try:
        rospy.init_node('obj_list_fusion')

        rate = rospy.Rate(10) # 10Hz

        D = obj_mapping()
        D.spawn_model("coke", "coke_can2", 0,0,0)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
