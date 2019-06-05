#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int32

from custom_msgs.msg import ObjDetected
from custom_msgs.msg import ObjDetectedList

from geometry_msgs.msg import Pose2D

from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

import numpy as np
import math

import time

import matplotlib.pyplot as plt

theta_imu = 0


class Auto_Nav:
    def __init__(self):


        self.theta_imu = 0
        self.obj_list = []
        self.activated = True
        self.state = 0
        self.ang = 0
        self.tx = 0
        self.distance = 0

        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber('/objects_detected', ObjDetectedList, self.objs_callback)
        self.angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)
        self.test = rospy.Publisher("test", Int32, queue_size=10)



    def punto_medio(self):

        distances = []
        Ys = []
        for i in range(len(self.obj_list)):
            distances.append(self.obj_list[i]['X'])
            Ys.append(self.obj_list[i]['Y'])

        idx1 = np.argsort(distances)[0]
        idx2 = np.argsort(distances)[1]
        
        z1 = distances[idx1]
        
        y1 = -1*Ys[idx1]

        z2 = distances[idx2]
        
        y2 = -1*Ys[idx2]
        
        zc = min([z1,z2]) + abs(z1 - z2)/2
        yc = min([y1,y2]) + abs(y1 - y2)/2
        self.distance = zc
        offset = .55
        
        yc = 0.00001 if yc == 0 else yc

        ang = math.atan((zc+offset)/yc)
        ang_rad = ang
        ang = math.degrees(ang)
        

        
        if ang < 0:
            ang = -1*(ang + 90)
        else:
            ang = 90 - ang
        
        ang_rad = math.radians(ang)
        
        
        #print(ang)
        
        ang_final = ang_rad + self.theta_imu
        

        self.angulo_pub.publish(ang_final)
        '''
        plt.clf()
        plt.plot(y1,z1, 'go',markersize=5)
        plt.plot(y2,z2, 'go',markersize=5)
        plt.plot(0,0,'ro')
        plt.plot(yc,zc,'r*')
        plt.axis([-5, 5, 0, 8])
        plt.pause(0.0001)
        plt.draw()
        '''

    def straight(self):
        self.angulo_pub.publish(self.theta_imu)

    def look_finding(self):
        self.tx = 10
        if self.distance < 7:
            self.tx = 7
        if self.distance < 0.5:
            self.tx = 0

        self.d_thrust_pub.publish(self.tx)
        delta = .17 if self.theta_imu < 0 else -.17
        self.angulo_pub.publish(self.theta_imu + delta)

    def ins_pose_callback(self,pose):
        self.theta_imu = pose.theta

    def objs_callback(self,data):
        print("a")
        self.obj_list = []
        for i in range(data.len):
            self.obj_list.append({'X' : data.objects[i].X, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})




    
if __name__ == '__main__':

    rospy.init_node('auto_nav', anonymous=True)
    rate = rospy.Rate(10)

    E = Auto_Nav()

    time.sleep(1)

    
    c = 0
    while E.activated:
        print(E.state)

        if E.state == 0:
            E.test.publish(0)
            if len(E.obj_list) >= 2:
                E.punto_medio()
                c = 0
            else:
                c = c + 1
            if c > 1000:
                E.state = 1
        
        if E.state == 1:
            E.test.publish(1)
            E.straight()
            time.sleep(3)
            E.state = 2

        if E.state == 2:
            E.test.publish(2)
            if len(E.obj_list) >= 2:
                E.state = 3
            else:
                E.look_finding()

        if E.state == 3:
            E.test.publish(3)
            if len(E.obj_list) >= 2:
                E.punto_medio()
                c = 0
            else:
                c = c + 1
            if c > 1000:
                E.state = 4

        if E.state == 4:
            E.test.publish(4)
            E.status_pub.publish(1)
            print('Fin')


        



    rospy.spin()


