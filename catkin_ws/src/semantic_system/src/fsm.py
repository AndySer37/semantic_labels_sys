#!/usr/bin/env python
import numpy as np

import roslib
import rospy
import smach
import smach_ros
from text_msgs.srv import *
from text_msgs.msg import *
from std_srvs.srv import Trigger, TriggerResponse,TriggerRequest, Empty, EmptyResponse, EmptyRequest
import rospkg
from cv_bridge import CvBridge, CvBridgeError

Initial_gripper = '/gripper_control/initial'
Detect_SRV = "/text_detection/text_detection"
Barcode_SRV = "/bb_ssd_prediction/barcode_detection"
# Pose_SRV = "/depth_to_pose/get_pose"
Object_Pose_SRV = "/object_pose_node"
Brand_name_Pose_SRV = "/bn_pose_node"

Move_srv = "/arm_control/move_to"
Home_srv = "/arm_control/home"

Initial = 0
Perception_bn = 1
pose_bn = 2
pick_bn = 3
Perception_obj = 4
pick_obj = 5
FLIP = 6
HOME = 7

STOP = -99
ERROR = -999


class FSM():
    def __init__(self):
        r = rospkg.RosPack()
        self.commodity_list = []
        self.read_commodity(r.get_path('text_msgs') + "/config/commodity_list.txt")

        self.last_state = STOP
        self.state = STOP
        self.last_img = 0
        self.last_depth = 0
        self.last_mask = 0
        self.last_count = 0
        self.last_list = []
        self.cv_bridge = CvBridge()
        self.mani_req = manipulationRequest()
        self.start = rospy.Service("~start", Trigger, self.srv_start)
        self.start = rospy.Service("~barcode_start", Trigger, self.barcode_start)

    def read_commodity(self, path):

        for line in open(path, "r"):
            line = line.rstrip('\n')
            self.commodity_list.append(line)
        print "Node (FSM): Finish reading list"

    def initial(self):
        self.mani_req = manipulationRequest()
        self.last_img = 0
        self.last_depth = 0
        self.last_mask = 0
        self.last_count = 0
        self.last_list = []

    def srv_start(self, req):

        self.state = Initial
        return TriggerResponse(success=True, message="Request accepted.")

    def process(self):

        if self.state == Initial:
            rospy.wait_for_service(Initial_gripper)
            try:
                gripper_initial_ser = rospy.ServiceProxy(Initial_gripper, Empty)
                req = EmptyRequest()
                resp1 = gripper_initial_ser(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            self.initial()

            emp = TriggerRequest()

            try:
                rospy.wait_for_service(Home_srv, timeout=10)
                home_srv = rospy.ServiceProxy(Home_srv, Trigger)
                home_resp = home_srv(emp)

            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            self.state = Perception_bn
            return 

        if self.state == Perception_bn:
            rospy.wait_for_service(Detect_SRV)
            try:
                perception_bn_ser = rospy.ServiceProxy(Detect_SRV, text_detection_srv)
                req = text_detection_srvRequest()
                resp1 = perception_bn_ser(req)
                self.last_img = resp1.image
                self.last_depth = resp1.depth
                self.last_mask = resp1.mask
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 

            upward_list = np.unique(self.cv_bridge.imgmsg_to_cv2(self.last_mask, "8UC1"))
            print "Brandname result: ", upward_list
            if len(upward_list) == 1:
                self.state = Perception_obj
                self.last_count = 0
                self.last_list = []
            else:
                self.state = pose_bn
                self.last_count = len(upward_list) - 1
                self.last_list = upward_list
            return 

        if self.state == Perception_obj:  
            object_req = object_onlyRequest()
            try:
                rospy.wait_for_service(Object_Pose_SRV, timeout=10)
                object_req.image = self.last_img
                object_req.depth = self.last_depth
                object_pose_srv = rospy.ServiceProxy(Object_Pose_SRV, object_only)
                recog_resp = object_pose_srv(object_req)

                print "Object num: ", recog_resp.count
                if recog_resp.count == 0:
                    self.state = STOP
                else:
                    self.mani_req.mode = "Object"
                    self.mani_req.object = "Non_known"
                    self.mani_req.pose = recog_resp.ob_list[0].pose
                    print self.mani_req.pose
                    self.state = HOME
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
            return 

        if self.state == pose_bn:  
            bn_req = bn_pose_srvRequest()
            bn_req.count = self.last_count
            for i in self.last_list:
                if i != 0:
                    bn_req.list.append(i)
            try:
                print bn_req.list
                rospy.wait_for_service(Brand_name_Pose_SRV, timeout=10)
                bn_req.total_list = len(self.commodity_list)
                bn_req.image = self.last_img
                bn_req.depth = self.last_depth
                bn_req.mask = self.last_mask
                bn_pose_est_srv = rospy.ServiceProxy(Brand_name_Pose_SRV, bn_pose_srv)
                recog_resp = bn_pose_est_srv(bn_req)

                if recog_resp.count == 0:
                    self.state = ERROR
                else:
                    direct = recog_resp.ob_list[0].object / len(self.commodity_list)
                    obj_name = self.commodity_list[recog_resp.ob_list[0].object % len(self.commodity_list)]
                    self.mani_req.mode = "BN"
                    self.mani_req.object = obj_name
                    self.mani_req.pose = recog_resp.ob_list[0].pose
                    print self.mani_req.pose
                    print "Picking object ", obj_name
                    print "Direction ", str(direct*90)
                    self.state = HOME
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
            return 


        if self.state == pick_bn:  
            try:
                rospy.wait_for_service(Move_srv, timeout=10)
                mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                mani_resp = mani_move_srv(self.mani_req)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
                
            self.gripper_off()
            self.state = HOME
            return       

        if self.state == pick_obj:  
            try:
                rospy.wait_for_service(Move_srv, timeout=10)
                mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                mani_resp = mani_move_srv(self.mani_req)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            self.gripper_off()
            self.state = HOME
            return   

        if self.state == HOME:  
            emp = TriggerRequest()
            try:
                rospy.wait_for_service(Home_srv, timeout=10)
                home_srv = rospy.ServiceProxy(Home_srv, Trigger)
                home_resp = home_srv(emp)

            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            self.state = STOP
            return   

    def gripper_off(self):
        rospy.sleep(1.5)
        rospy.wait_for_service('/gripper_control/close')
        try:
            gripper_close_ser = rospy.ServiceProxy('/gripper_control/close', Empty)
            req = EmptyRequest()
            resp1 = gripper_close_ser(req)
            rospy.sleep(1.5)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e       

    def barcode_start(self, req111):


        rospy.wait_for_service(Barcode_SRV)
        try:
            perception_barcode_ser = rospy.ServiceProxy(Barcode_SRV, text_detection_srv)
            req = text_detection_srvRequest()
            resp1 = perception_barcode_ser(req)
            self.last_img = resp1.image
            self.last_depth = resp1.depth
            self.last_mask = resp1.mask
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e 

        upward_list = np.unique(self.cv_bridge.imgmsg_to_cv2(self.last_mask, "8UC1"))
        print "Brandname result: ", upward_list
        if len(upward_list) == 1:
            print "No Barcode Detected !!"
        else:
            self.last_count = len(upward_list) - 1
            self.last_list = upward_list
        
        bn_req = bn_pose_srvRequest()
        bn_req.count = self.last_count
        for i in self.last_list:
            if i != 0:
                bn_req.list.append(i)
        try:
            print bn_req.list
            rospy.wait_for_service(Brand_name_Pose_SRV, timeout=10)
            bn_req.total_list = len(self.commodity_list)
            bn_req.image = self.last_img
            bn_req.depth = self.last_depth
            bn_req.mask = self.last_mask
            bn_pose_est_srv = rospy.ServiceProxy(Brand_name_Pose_SRV, bn_pose_srv)
            recog_resp = bn_pose_est_srv(bn_req)

            if recog_resp.count == 0:
                print "Error"
            else:
                self.mani_req.mode = "Barcode"
                self.mani_req.object = "Barcode"
                self.mani_req.pose = recog_resp.ob_list[0].pose
                print self.mani_req.pose
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e 



    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")

if __name__ == '__main__':
    rospy.init_node('fsm')
    foo = FSM()
    rospy.on_shutdown(foo.shutdown_cb)
    while not rospy.is_shutdown():
        foo.process()
        rospy.sleep(2)

    rospy.spin()