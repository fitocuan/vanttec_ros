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
        self.current = lambda: int(round(time.time()*1000))
        self.InitTime = self.current()
        self.ang_change = 0
        self.ang_change2 = 0



        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber('/objects_detected', ObjDetectedList, self.objs_callback)
        self.angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.status_pub = rospy.Publisher("status", Int32, queue_size=10)
        self.test = rospy.Publisher("test", Int32, queue_size=10)


    def curr_time(self):
        curTime = self.current()
        difTime = curTime - self.InitTime
        realTime = difTime/float(1000)
        timer = '{:.3f}'.format(realTime)
        return float(timer)

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
        self.ang = -1 if ang_rad < 0 else 1

        ang_final = ang_rad + self.theta_imu
        

        #self.angulo_pub.publish(ang_final)
        self.tx = 10
        if self.distance < 7:
            self.tx = 7
        if self.distance < 0.5:
            self.tx = 0

        #self.d_thrust_pub.publish(self.tx)

        if abs(z1 - z2) <= 5:
            self.desired(self.tx, ang_final)
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
        self.tx = 10

        self.desired(self.tx, self.theta_imu)

    def look_finding(self):

        #.1 = 6 grados

        self.tx = 5
        delta = .1


        
        if self.ang == 1:
            if self.ang_change < .22 :
                self.ang_change = self.ang_change + delta
                self.desired(self.tx, self.theta_imu - delta)
            else:
                self.ang_change = 0
                self.ang = -1

        else:
            if self.ang_change < .22 :
                self.ang_change = self.ang_change + delta
                self.desired(self.tx, self.theta_imu + delta)
            else:
                self.ang_change = 0
                self.ang = 1

        

        time.sleep(2)

    def ins_pose_callback(self,pose):
        self.theta_imu = pose.theta

    def objs_callback(self,data):
        print("a")
        self.obj_list = []
        for i in range(data.len):
            self.obj_list.append({'X' : data.objects[i].X, 'Y' : data.objects[i].Y, 'color' : data.objects[i].color, 'class' : data.objects[i].clase})


    def desired(self, thrust, heading):
        self.angulo_pub.publish(heading)
        self.d_thrust_pub.publish(thrust)

    def enderezar(self,curr_angle):

        self.tx = 0.5
        delta = 0.1
        total_delta = 0
        while True:
            if curr_angle > 0:
                self.desired(self.tx, self.theta_imu - delta)
            else:
                self.desired(self.tx, self.theta_imu + delta)

            total_delta += delta

            time.sleep(0.5)

            if abs(curr_angle) < delta:
                break

        


    
if __name__ == '__main__':

    rospy.init_node('auto_nav', anonymous=True)
    rate = rospy.Rate(10)
    E = Auto_Nav()
    
    while E.activated:
        print(E.state)

        if E.state == 0:
            E.test.publish(0)
            if len(E.obj_list) >= 2:
                E.punto_medio()
            else:
                initTime = E.curr_time()
                while len(E.obj_list) < 2:
                    if E.curr_time() - initTime > 3:
                        E.state = 1
                        curr_angle = E.theta_imu
                        break
        
        if E.state == 1:
            E.test.publish(1)
            E.enderezar(curr_angle)
            E.straight()
            time.sleep(3)
            E.state = 4


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
            else:
                initTime = E.curr_time()
                while len(E.obj_list) < 2:
                    if E.curr_time() - initTime > 3:
                        E.state = 4
                        break

        if E.state == 4:
            E.test.publish(4)
            E.desired(0,E.theta_imu)
            time.sleep(1)
            E.status_pub.publish(1)
            print('Fin')


        



    rospy.spin()


