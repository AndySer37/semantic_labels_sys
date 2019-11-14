#!/usr/bin/env python2.7
import numpy as np
from math import pi, sqrt
import sys
import copy
import random
import copy

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from std_msgs.msg import String, Float64, Bool, Header
from std_srvs.srv import Trigger, TriggerResponse, Empty
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from arm_operation.srv import *
# from arm_operation.msg import *
from text_msgs.srv import *
from text_msgs.msg import *

# Initial robot pose
JOINTS_PICKING_HOME = [6.2063, -2.65049, 2.1060, -2.5701, -1.50571, 1.573]

JOINTS_PLACING_HOME = [2.35891, -1.62645, 1.4686, -1.37149, -1.57006, -0.78227]
JOINTS_SCANNING = [2.78776478767395, -2.512611214314596, 2.008678436279297,
                   -1.7134316603290003, -1.3840788046466272, -0.27180654207338506]

# Placing related stuff
LARGE_CUBOID_PLACING_OFFSET = (+0.03, -0.14)
MEDIUM_CUBOID_PLACING_OFFSET = (+0.13, -0.14)
CYLINDER_PLACING_OFFSET = (+0.23, -0.14)
PLACING_TAG_ID = 1
FIXED_PLACING_HEIGHT = 0.23


class FSM(object):
    """docstring for FSM"""
    def __init__(self):


        # TF listener
        self.tf_listener = tf.TransformListener()

        # ROS publisher & subscriber & service
        # self.pub_target_pose = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.home_srv = rospy.Service("process", Empty, self.process_cb)
        # self.image_collect_srv = rospy.Service("m_image_collect", Trigger, self.image_collect_cb)
        # self.trigger_srv = rospy.Service("m_pick", Trigger, self.pick_cb)
        # self.loc_srv = rospy.Service("m_localize", Trigger, self.loc_cb)
        # self.calibrate_srv = rospy.Service("m_calibrate", Trigger, self.calibrate_cb)
        # self.pose_srv = rospy.Service("m_pose", Trigger, self.pose_cb)


    def process_cb(self, req):
        # Update FSM state

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            # msg = joint_value()
            for i in range(6):
                req.joint[i] = JOINTS_PICKING_HOME[i]
            # req.joints.append(msg)
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        rospy.wait_for_service('/object_pose_node')
        res_pose = object_onlyResponse()
        try:
            pose_ser = rospy.ServiceProxy('/object_pose_node', object_only)
            req = object_onlyRequest()
            res_pose = pose_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print res_pose



        # rospy.wait_for_service('/ur5_control_server/ur_control/go_straight')
        # try:
        #     ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/go_straight', target_pose)
        #     req = target_poseRequest()
        #     msg = Pose()
        #     for i in range(6):
        #         msg[i] = JOINTS_PICKING_HOME[i]
        #     req.joints.append(msg)
        #     resp1 = ur5_joint_ser(req)
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e


    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('system_fsm', anonymous=False)
    node = FSM()
    rospy.on_shutdown(node.shutdown_cb)
    rospy.spin()
