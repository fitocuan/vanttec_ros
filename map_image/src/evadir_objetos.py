#!/usr/bin/env python


import rospy
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np




class evadir_objectos:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth = np.zeros((560,1000,3),np.uint8)
        rospy.Subscriber("/zed/depth/depth_registered", Image, self.callback_zed_depth)
        self.pub = rospy.Publisher('/obstacles', String, queue_size=10)
        #rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.callback_zed_depth)





    def callback_zed_depth(self,img):
        img = self.bridge.imgmsg_to_cv2(img)
        H, W = img.shape[:2]
        y = (H/5)*2
        x = W/4
        h = (H/5)
        w = (W/4)*2


        
        ret,img = cv2.threshold(img,3,255,cv2.THRESH_BINARY)
        
        img = img[y:y+h,x:x+w]


        H, W = img.shape[:2]
        y = 0
        x = 0
        h = H
        w = W
        c1 = img[y:y+h,x:x+(w/4)]
        c2 = img[y:y+h,x+w/4:x+w/4*2]
        c3 = img[y:y+h,x+w/4*2:x+w/4*3]
        c4 = img[y:y+h,x+w/4*3:x+w/4*4]
        cv2.rectangle(img, (x,y),(x+(w/4),y+h), (255,255,0), 2)

        w_c1 = 1 if round(cv2.countNonZero(c1)/(H*W/4.0),3) < 0.9 else 0
        w_c2 = 1 if round(cv2.countNonZero(c2)/(H*W/4.0),3) < 0.9 else 0
        w_c3 = 1 if round(cv2.countNonZero(c3)/(H*W/4.0),3) < 0.9 else 0
        w_c4 = 1 if round(cv2.countNonZero(c4)/(H*W/4.0),3) < 0.9 else 0

        ret = str(w_c1)+str(w_c2)+str(w_c3)+str(w_c4)

        print(ret)
        self.pub.publish(ret)

        cv2.imshow("a", img)
        cv2.waitKey(3)
        self.depth = img
        







    

if __name__ == '__main__':
    try:
        rospy.init_node('evadir_objectos')
        rate = rospy.Rate(5)
        E = evadir_objectos()
        rospy.spin()


    except rospy.ROSInterruptException:
        pass