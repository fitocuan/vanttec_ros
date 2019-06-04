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

        self.dt = 0
        self.dh = 0

        self.reference_heading = 0

        rospy.Subscriber("ref", Pose2D, self.ref_callback)

        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)

    def ref_callback(self, refh):
        self.reference_heading = refh.theta

    def desired(self, thrust, heading):
        #while not rospy.is_shutdown():
        self.dt = thrust
        self.dh = heading
        self.d_thrust_pub.publish(self.dt)
        self.d_heading_pub.publish(self.dh)

def main():
    rospy.init_node('ch', anonymous=True)
    t = Test()
    if t.testing:
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 1:
            t.desired(0,t.reference_heading)
            time.sleep(0.1)
        while (rospy.Time.now().secs - start_time) <= 45:
            t.desired(15,t.reference_heading)
            time.sleep(0.1)
        t.desired(0,t.reference_heading)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass