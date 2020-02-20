#!/usr/bin/env python
import numpy as np

import roslib
import rospy
import smach
import smach_ros
import tf
from text_msgs.srv import *
from text_msgs.msg import *
from std_srvs.srv import Trigger, TriggerResponse,TriggerRequest, Empty, EmptyResponse, EmptyRequest
from arm_operation.srv import *
from arm_operation.msg import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion 
from converter import  *
import copy

Initial_gripper = '/gripper_control/initial'
Detect_SRV = "/text_detection/text_detection"
Barcode_SRV = "/bb_ssd_prediction/barcode_detection"
# Pose_SRV = "/depth_to_pose/get_pose"
Object_Pose_SRV = "/object_pose_node"
Brand_name_Pose_SRV = "/bn_pose_node"
Get_pose = "/ur5_control_server/ur_control/get_tcp_pose"
Goto_joint = "/ur5_control_server/ur_control/goto_joint_pose"

## Arm_process
Move_srv = "/arm_control/move_to"
Home_srv = "/arm_control/home"
Flip_srv = "/arm_control/flip"
Toss_srv = "/arm_control/toss"
Suck_srv = "/arm_control/suck_process"

## Data science 
Raisin_srv = "/arm_control/raisin"
Crayon_srv = "/arm_control/crayon"

## topic
Joint_value = "/joint_states"

Initial = 0
Perception_bn = 1
pose_bn = 2
pick_bn = 3
Perception_obj = 4
pick_obj = 5
FLIP = 6
HOME = 7
Prepare_Pick = 8
Rotate = 9
Place_On_Shelf = 10
STOP = -99
ERROR = -999
###
Place_raisin = 20
Place_crayon = 21

### commodity_list
### background crayons kleenex vanish milo raisins andes pocky lays hunts 3m
OBJ_HEIGHT = [0.0, -0.038, -0.005, 0.0, 0.005, -0.03, -0.02, 0.045, 0.103, 0.01, 0.115]
Y_DIS = 0.23
OBJ_Depth = [0.0, 0.005, 0.0, -0.02, 0.0, 0.0, 0.005, 0.0, -0.02, -0.02, -0.025]

### Static Joint
PrePare_Place = [5.506424903869629, -1.7503469626056116, 1.9935364723205566, -1.8246658484088343, -1.5188863913165491, 0.6822109818458557]

class FSM():
    def __init__(self):
        r = rospkg.RosPack()
        self.commodity_list = []
        self.shelf_list = []
        self.read_commodity(r.get_path('text_msgs') + "/config/commodity_list.txt")
        self.repeat_bn_detect = 5
        self.bn_detect_count = 0
        self.last_state = STOP
        self.state = STOP
        self.test_without_arm = False
        self.last_img = 0
        self.last_depth = 0
        self.last_mask = 0
        self.last_count = 0
        self.last_list = []
        self.cv_bridge = CvBridge()
        self.mani_req = manipulationRequest()
        self.rot = 0
        self.object = ""
        self.start = rospy.Service("~start", Trigger, self.srv_start)
        self.reset_shelf = rospy.Service("~reset_shelf", Trigger, self.reset_shelf)
        self.start_bar = rospy.Service("~barcode_start", Trigger, self.barcode_start)
        self.x_180 = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]],dtype = np.float32)
        self.x_90 = np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]],dtype = np.float32)

        self.shelf_pose = Pose()
        self.shelf_pose.position.x = 0.139651686266; self.shelf_pose.position.y = -0.384602086405; self.shelf_pose.position.z = 0.424965406054
        self.shelf_pose.orientation.x = 0.0038249214696; self.shelf_pose.orientation.y = -0.00640644391886
        self.shelf_pose.orientation.z = -0.68676603623; self.shelf_pose.orientation.w = 0.726840243061
        self.shelf_count = 0

        self.place_shelf_test = rospy.Service("~test_arm_place_shelf", Trigger, self.test_arm)

    def test_arm(self, req):

        temp_mani_req = manipulationRequest()
        temp_mani_req.pose = copy.deepcopy(self.shelf_pose)
        temp_mani_req.pose.position.x += (self.shelf_count * 0.11)
        try:
            rospy.wait_for_service(Move_srv, timeout=10)
            mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
            mani_resp = mani_move_srv(temp_mani_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e
        rospy.sleep(0.3)

        temp_mani_req.pose.position.y -= Y_DIS
        try:
            rospy.wait_for_service(Move_srv, timeout=10)
            mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
            mani_resp = mani_move_srv(temp_mani_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e
        rospy.sleep(0.3)

        temp_mani_req.pose.position.y += Y_DIS
        try:
            rospy.wait_for_service(Move_srv, timeout=10)
            mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
            mani_resp = mani_move_srv(temp_mani_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e
        rospy.sleep(0.3)

        self.shelf_count += 1
        return TriggerResponse(success=True, message="Request accepted.")

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
        self.rot = 0
        self.bn_detect_count = 0
        self.last_list = []
        self.object = ""

    def srv_start(self, req):

        self.state = Initial
        self.last_state = Initial
        return TriggerResponse(success=True, message="Request accepted.")

    def reset_shelf(self, req):
        self.shelf_count = 0
        self.shelf_list = []
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

            emp = TriggerRequest()
            try:
                rospy.wait_for_service(Home_srv, timeout=10)
                home_srv = rospy.ServiceProxy(Home_srv, Trigger)
                home_resp = home_srv(emp)

            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
            self.initial()
            self.state = Perception_bn
            return 

        if self.state == Perception_bn:
            self.bn_detect_count += 1
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
            if len(upward_list) == 1 and self.bn_detect_count >= self.repeat_bn_detect:
                self.state = HOME   ### Perception_obj
                self.last_count = 0
                self.last_list = []
            elif len(upward_list) > 1:
                self.bn_detect_count = 0
                self.state = pose_bn
                self.last_count = len(upward_list) - 1
                self.last_list = upward_list
            self.last_state = Perception_bn
            ### Test obj
            self.state = Perception_obj
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
                    self.last_state = Perception_obj
                    self.state = STOP
                else:
                    self.mani_req.mode = "Object"
                    self.mani_req.object = "Non_known"
                    self.mani_req.pose = recog_resp.ob_list[0].pose
                    q_pose = Quaternion(self.mani_req.pose.orientation.w, self.mani_req.pose.orientation.x, self.mani_req.pose.orientation.y, self.mani_req.pose.orientation.z)
                    q_mat = q_pose.rotation_matrix
                    trans = np.array([0,0,-0.05],dtype = np.float32)
                    trans = np.dot(q_mat,trans)
                    self.mani_req.pose.position.x += trans[0]
                    self.mani_req.pose.position.y += trans[1]
                    self.mani_req.pose.position.z += trans[2]
                    print self.mani_req.pose
                    if self.test_without_arm:
                        self.state = HOME
                    else:
                        self.state = Prepare_Pick
                    self.last_state = Perception_obj
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
                    self.last_state = pose_bn
                    self.state = ERROR
                else:
                    direct = recog_resp.ob_list[0].object / len(self.commodity_list)
                    self.object = self.commodity_list[recog_resp.ob_list[0].object % len(self.commodity_list)]
                    self.mani_req.mode = "BN"
                    self.mani_req.object = self.object
                    self.mani_req.pose = recog_resp.ob_list[0].pose
                    self.rot = direct
                    q_pose = Quaternion(self.mani_req.pose.orientation.w, self.mani_req.pose.orientation.x, self.mani_req.pose.orientation.y, self.mani_req.pose.orientation.z)
                    q_mat = q_pose.rotation_matrix 
                    rpy = rot_to_rpy(q_mat)
                    # print "old =====" ,rpy
                    inv = -1 if np.abs(rpy[0]) > math.pi/2 else 1
                    q_new_mat = np.dot(q_mat, rpy_to_rot([0, inv*(math.pi/2 - rpy[1]), 0]))
                    if self.rot >= 2:
                        q_new_mat = np.dot(q_new_mat, rpy_to_rot([math.pi, 0, 0]))
                    # print "old222 =====" ,rot_to_rpy(q_new_mat)
                    q_new_pose = Quaternion(matrix=q_new_mat)
                    self.mani_req.pose.orientation.w = q_new_pose.w
                    self.mani_req.pose.orientation.x = q_new_pose.x
                    self.mani_req.pose.orientation.y = q_new_pose.y
                    self.mani_req.pose.orientation.z = q_new_pose.z  
                    self.pub_tf(self.mani_req.pose, "rectify_rot")
                    print self.mani_req.pose
                    print "Picking object ", self.object
                    print "Direction ", str(direct*90)
                    if self.test_without_arm:
                        self.state = HOME
                    else:
                        self.state = Prepare_Pick
                    self.last_state = pose_bn
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
            return 
##################################################################################################################
        # if self.state == Place_raisin:
        #     emp = TriggerRequest()
        #     try:
        #         rospy.wait_for_service(Raisin_srv, timeout=10)
        #         raisin_srv = rospy.ServiceProxy(Raisin_srv, Trigger)
        #         emp_resp = raisin_srv(emp)
        #     except (rospy.ServiceException, rospy.ROSException), e:
        #         print "Service call failed: %s"%e 
        #     self.last_state = Place_raisin
        #     self.state = HOME
        #     return

        # if self.state == Place_crayon:
        #     emp = TriggerRequest()
        #     try:
        #         rospy.wait_for_service(Crayon_srv, timeout=10)
        #         crayon_srv = rospy.ServiceProxy(Crayon_srv, Trigger)
        #         emp_resp = crayon_srv(emp)
        #     except (rospy.ServiceException, rospy.ROSException), e:
        #         print "Service call failed: %s"%e 
        #     self.last_state = Place_crayon
        #     self.state = HOME
        #     return

##################################################################################################################
        if self.state == Prepare_Pick:
            
            self.mani_req.pose.position.z += 0.06
            try:
                rospy.wait_for_service(Move_srv, timeout=10)
                mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                mani_resp = mani_move_srv(self.mani_req)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            self.mani_req.pose.position.z -= 0.06
            
            if self.last_state == Perception_obj:
                self.mani_req.pose.position.z += 0.01
                self.state = pick_obj
            elif self.last_state == pose_bn:
                    self.state = pick_bn 
            self.last_state = Prepare_Pick
            return              

        if self.state == pick_bn:  
            self.mani_req.pose.position.z += OBJ_Depth[self.commodity_list.index(self.object)]
            try:
                rospy.wait_for_service(Move_srv, timeout=10)
                mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                mani_resp = mani_move_srv(self.mani_req)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
                
            self.gripper_off()
            if self.rot >= 2:
                self.state = Rotate
            # elif self.object == "raisins":
            #     self.state = Place_raisin
            # elif self.object == "crayons":
            #     self.state = Place_crayon
            else: 
                self.state = Place_On_Shelf
            self.last_state = pick_bn
            return       

        if self.state == Place_On_Shelf:
            if len(self.shelf_list) > 5:
                rospy.loginfo("Shelf is out of room!")
                # self.state = HOME
            else:
                if self.object in self.shelf_list:
                    addr = self.shelf_list.index(self.object)
                else:
                    addr = len(self.shelf_list)
                    self.shelf_list.append(self.object)

                rospy.wait_for_service(Goto_joint)
                try:
                    ur5_joint_ser = rospy.ServiceProxy(Goto_joint, joint_pose)
                    req = joint_poseRequest()
                    msg = joint_value()
                    for i in range(6):
                        msg.joint_value[i] = PrePare_Place[i]
                    req.joints.append(msg)
                    req.factor = 0.5
                    resp1 = ur5_joint_ser(req)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
                temp_mani_req = manipulationRequest()

                temp_mani_req.pose = copy.deepcopy(self.shelf_pose)
                temp_mani_req.pose.position.x += (addr * 0.11)
                temp_mani_req.pose.position.z += OBJ_HEIGHT[self.commodity_list.index(self.object)]
                try:
                    rospy.wait_for_service(Move_srv, timeout=10)
                    mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                    mani_resp = mani_move_srv(temp_mani_req)
                except (rospy.ServiceException, rospy.ROSException), e:
                    print "Service call failed: %s"%e
                rospy.sleep(0.3)

                temp_mani_req.pose.position.y -= Y_DIS
                try:
                    rospy.wait_for_service(Move_srv, timeout=10)
                    mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                    mani_resp = mani_move_srv(temp_mani_req)
                except (rospy.ServiceException, rospy.ROSException), e:
                    print "Service call failed: %s"%e
                rospy.sleep(0.3)
                self.gripper_open()
                temp_mani_req.pose.position.y += Y_DIS
                try:
                    rospy.wait_for_service(Move_srv, timeout=10)
                    mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                    mani_resp = mani_move_srv(temp_mani_req)
                except (rospy.ServiceException, rospy.ROSException), e:
                    print "Service call failed: %s"%e
                rospy.sleep(0.3)
                
            self.state = HOME
            self.last_state = Place_On_Shelf

        if self.state == Rotate:
            self.rotation()
            self.last_state = Rotate
            self.state = HOME
            return

        if self.state == pick_obj:  
            try:
                rospy.wait_for_service(Move_srv, timeout=10)
                mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                mani_resp = mani_move_srv(self.mani_req)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            try:
                emp = TriggerRequest()
                rospy.wait_for_service(Suck_srv, timeout=10)
                suck = rospy.ServiceProxy(Suck_srv, Trigger)
                emp_resp = suck(emp)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 
            self.mani_req.pose.position.z += 0.12
            try:
                rospy.wait_for_service(Move_srv, timeout=10)
                mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
                mani_resp = mani_move_srv(self.mani_req)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            self.last_state = pick_obj
            self.state = FLIP
            return   

        if self.state == FLIP:  
            print self.mani_req.pose.position.z
            emp = TriggerRequest()
            try:
                rospy.wait_for_service(Flip_srv, timeout=10)
                flip_srv = rospy.ServiceProxy(Flip_srv, Trigger)
                emp_resp = flip_srv(emp)
            except (rospy.ServiceException, rospy.ROSException), e:
                print "Service call failed: %s"%e 

            self.last_state = FLIP
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

            if self.last_state == pick_bn:
                self.gripper_open()
                self.state = STOP
            elif self.last_state == FLIP or self.last_state == Rotate or self.test_without_arm:
                self.state = Perception_bn
            else:
                self.gripper_open()
                self.state = STOP

            self.last_state = HOME
            return   

    def rotation(self):
        # print "sssssss", joint.position[5]

        self.mani_req.pose.position.z += 0.03
        try:
            rospy.wait_for_service(Move_srv, timeout=10)
            mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
            mani_resp = mani_move_srv(self.mani_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e 

        joint = rospy.wait_for_message(Joint_value, JointState, timeout=None)
        # joint.position[5] = 1.8
        rospy.wait_for_service(Goto_joint)
        try:
            ur5_joint_ser = rospy.ServiceProxy(Goto_joint, joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = joint.position[i]
            msg.joint_value[5] = 2.2
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        

        self.gripper_open()

        try:
            ur5_joint_ser = rospy.ServiceProxy(Goto_joint, joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = joint.position[i]
            msg.joint_value[1] -= 0.2
            msg.joint_value[5] = 1.3
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e 

        self.mani_req.pose.position.z += 0.1
        try:
            rospy.wait_for_service(Move_srv, timeout=10)
            mani_move_srv = rospy.ServiceProxy(Move_srv, manipulation)
            mani_resp = mani_move_srv(self.mani_req)
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e        


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

    def gripper_open(self):
        # rospy.sleep(1.5)
        rospy.wait_for_service('/gripper_control/open')
        try:
            gripper_close_ser = rospy.ServiceProxy('/gripper_control/open', Empty)
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

    def pub_tf(self, pose, frame_name):
        q_pose = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        q_mat = q_pose.rotation_matrix 
        rpy = rot_to_rpy(q_mat)
        print "new =====" ,rpy
        br = tf.TransformBroadcaster()
        br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                        tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]),
                        rospy.Time.now(),
                        frame_name,
                        "base_link")    

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