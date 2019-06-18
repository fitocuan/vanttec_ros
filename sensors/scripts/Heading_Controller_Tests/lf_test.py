#!/usr/bin/env python

import os
import time
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64

#Constant Thrust and Constant Heading
#15 N and imu rads
class Test:
    def __init__(self):
        self.testing = True

        self.theta_imu = 0
        self.tx = 0
        self.ang = 0
        self.ang_change = 0
        self.detectedobj = 0

        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber("detected", Float64, self.detect_callback)
        self.angulo_pub = rospy.Publisher('desired_heading', Float64, queue_size=10)
        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)

    def look_finding(self, angle):

        #.1 = 6 grados

        self.tx = 3
        delta = .0035

        if self.ang == 0:
            if self.ang_change < .15 :
                self.ang_change = self.ang_change + delta
                dh = angle + self.ang_change
                #self.theta_imu -= delta
                self.desired(self.tx, dh)
            else:
                self.ang_change = 0
                self.ang = 1

        elif self.ang == 1:
            if self.ang_change < .3 :
                self.ang_change = self.ang_change + delta
                #self.theta_imu -= delta
                dh = angle - self.ang_change
                self.desired(self.tx, dh)
            else:
                self.ang_change = 0
                self.ang = -1

        elif self.ang == -1:
            if self.ang_change < .3 :
                self.ang_change = self.ang_change + delta
                #self.theta_imu += delta
                dh = angle + self.ang_change
                self.desired(self.tx, dh)
            else:
                self.ang_change = 0
                self.ang = 1
        print self.ang
        time.sleep(0.1)

    def ins_pose_callback(self,pose):
        self.theta_imu = pose.theta

    def detect_callback(self,pose):
        self.detectedobj = pose.data

    def desired(self, thrust, heading):
        self.angulo_pub.publish(heading)
        self.d_thrust_pub.publish(thrust)

def main():
    rospy.init_node('lf', anonymous=True)
    t = Test()
    t.ang = 1
    imu = t.theta_imu
    while t.testing:
        if t.detectedobj == 0:
            t.look_finding(imu)
        else:
            t.testing = False
            t.desired(0, 0)
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass