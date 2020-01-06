#!/usr/bin/env python
import cv2
import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *
import serial
import time
import rospkg
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse, EmptyRequest
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class saver(object):
    def __init__(self):
        r = rospkg.RosPack()
        self.cv_bridge = CvBridge() 
        self.path = r.get_path('textsnake')

        self.save_640 = rospy.Service("~save_640", Empty, self.save_640)
        self.save_1280 = rospy.Service("~save_1280", Empty, self.save_1280)
        self.count = rospy.Service("~count_1", Empty, self.count_1)

        self.p_img_640 = os.path.join(self.path, "saver", "img_640")
        if not os.path.exists(self.p_img_640):
            os.makedirs(self.p_img_640)        

        self.p_img_1280 = os.path.join(self.path, "saver", "img_1280")
        if not os.path.exists(self.p_img_1280):
            os.makedirs(self.p_img_1280)
        
        self.saver_count = 0
        

    def save_640(self, req):
        img_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=None)
        depth = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=None)

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv_depth = self.cv_bridge.imgmsg_to_cv2(depth, "16UC1")

        cv2.imwrite("{}/{}.png".format(self.p_img_640, self.saver_count), cv_image)
        cv2.imwrite("{}/depth_{}.png".format(self.p_img_640, self.saver_count), cv_depth)

        rospy.loginfo("Saving 640x480 img.")

        return EmptyResponse()

    def save_1280(self, req):
        img_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=None)
        depth = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=None)

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv_depth = self.cv_bridge.imgmsg_to_cv2(depth, "16UC1")

        cv2.imwrite("{}/{}.png".format(self.p_img_1280, self.saver_count), cv_image)
        cv2.imwrite("{}/depth_{}.png".format(self.p_img_1280, self.saver_count), cv_depth)

        rospy.loginfo("Saving 1280x720 img.")
        return EmptyResponse()

    def count_1(self, req):
        self.saver_count += 1
        rospy.loginfo("Count {}!".format(self.saver_count))
        return EmptyResponse()

    def onShutdown(self):
        rospy.loginfo("Vacuum Control Node Shutdown.")  

if __name__ == '__main__': 
    rospy.init_node('saver',anonymous=False)
    saver = saver()
    rospy.on_shutdown(saver.onShutdown)
    rospy.spin()
