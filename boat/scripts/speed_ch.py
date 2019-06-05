#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList

from geometry_msgs.msg import Pose2D

from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

import numpy as np
import math

import matplotlib.pyplot as plt

EARTH_RADIUOS = 6371000


class Speed_Challenge:
    def __init__(self):
        self.obj_list = []
        self.activated = True
        self.state = 0
        self.theta_imu = 0
        self.lat = 0
        self.lon = 0
        
        
        rospy.Subscriber('/objects_detected', ObjDetectedList, self.callback)
        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)

    def ins_pose_callback(self, data):
        self.theta_imu = data.theta
        self.lat = data.x
        self.lon = data.y

    def gps_point_trans(self,y,x):
        p = np.array([x,y])
        J = np.array([[math.cos(self.theta_imu), -1*math.sin(self.theta_imu)],[math.sin(self.theta_imu), math.cos(self.theta_imu)]])
        n = J.dot(p)
        print(n)

        phi1 = math.radians(self.lat)

        latitude2  = self.lat  + (y / EARTH_RADIUOS) * (180 / math.pi)
        longitude2 = self.lon + (x / EARTH_RADIUOS) * (180 / math.pi) / math.cos(phi1 * math.pi/180)
        print( {
            'latitude': latitude2,
            'longitud': longitude2
        })



    def punto_medio(self):

        sortList = sorted(self.obj_list, key=lambda k: k['X'])
        
        if (sortList[0]['color'] == 'yellow' or sortList[0]['color'] == 'green' or sortList[0]['color'] == 'orange') and (sortList[1]['color'] == 'yellow' or sortList[0]['color'] == 'orange' or sortList[1]['color'] == 'green' and (sortList[0]['X'] != 'nan' or sortList[0]['X'] != 'nan')):
            print('Empezo Puntomedio')
            z1 = sortList[0]['X']
            y1 = -1*sortList[0]['Y']

            z2 = sortList[1]['X']
            y2 = -1*sortList[1]['Y']
            
            zc = min([z1,z2]) + abs(z1 - z2)/2
            yc = min([y1,y2]) + abs(y1 - y2)/2
            yc = 0.00001 if yc == 0 else yc

            print(zc,yc)
            self.gps_point_trans(yc,zc)

            plt.clf()
            plt.plot(y1,z1, 'go',markersize=5)
            plt.plot(y2,z2, 'go',markersize=5)
            plt.plot(0,0,'ro')
            plt.plot(yc,zc,'r*')
            plt.axis([-5, 5, 0, 8])
            plt.pause(0.0001)
            plt.draw()
            print('Termino Puntomedio')
            self.state = 1

        
    def waypoints_vuelta(self):
        if len(self.obj_list) == 1 :
            if (self.obj_list[0]['color'] == 'blue'):
                print('Empezo waypoints')
                v_x = self.obj_list[0]['X']
                v_y = self.obj_list[0]['Y']

                w1 = (v_x,v_y+1.5)
                w2 = (v_x+1.5,v_y)
                w3 = (v_x,v_y-1.5)

                plt.clf()
                plt.plot(0,0,'ro')
                plt.plot(v_y,v_x,'go')
                plt.plot(w1[1],w1[0],'r*')
                plt.plot(w2[1],w2[0],'r*')
                plt.plot(w3[1],w3[0],'r*')
                plt.axis([-5, 5, 0, 8])
                plt.pause(0.0001)
                plt.draw()

                print(w1,w2,w3)
                print('Termino waypoints')
                self.state = 2


    def callback(self,data):
        self.obj_list = []
        for i in range(data.len):
            self.obj_list.append({'X' : data.objects[i].X, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})



    
if __name__ == '__main__':

    rospy.init_node('pm', anonymous=True)
    rate = rospy.Rate(10)
    
    E = Speed_Challenge()
    while E.activated :
        if E.state == 0:
            if len(E.obj_list) >= 2:
                
                E.punto_medio()
                
        if E.state == 1:
            
            if len(E.obj_list) == 1 :
                
                E.waypoints_vuelta()
                
        if E.state == 2:
            E.activated = False
            print('Termino')
        

    #rospy.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2, callback_zed_cp)
    rospy.spin()
