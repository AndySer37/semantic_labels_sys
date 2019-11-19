#!/usr/bin/env python2.7
import numpy as np
from math import pi, sqrt
import sys
import copy
import random
import copy

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import String, Float64, Bool, Header
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse, EmptyRequest
from apriltags_ros.msg import AprilTagDetectionArray, AprilTagDetection
from arm_operation.srv import *
from arm_operation.msg import *
from text_msgs.srv import *
from text_msgs.msg import *
from pyquaternion import Quaternion 
from converter import rpy_to_rot, rot_to_rpy
from scipy.spatial.transform import Rotation as R

# Initial robot pose
JOINTS_PICKING_HOME = [6.2063, -2.65049, 2.1060, -2.5701, -1.50571, 0]
Prepare_To_Pick = [5.592137813568115, -2.750432793294088, 2.1252174377441406, -2.5681851545916956, -1.5061362425433558, 0]

Pick_UP_Final = [6.203226566314697, -2.5497353712665003, 1.7531695365905762, -2.55903131166567, -1.505477253590719, 2.3968450477696024e-05]
# Placing related stuff
LARGE_CUBOID_PLACING_OFFSET = (+0.03, -0.14)
MEDIUM_CUBOID_PLACING_OFFSET = (+0.13, -0.14)
CYLINDER_PLACING_OFFSET = (+0.23, -0.14)
PLACING_TAG_ID = 1
FIXED_PLACING_HEIGHT = 0.23


a1 = [6.158311367034912, -1.954024616871969, 0.8735723495483398, -1.3178542296039026, -1.5869110266314905, 0]
a2 = [5.789125919342041, -1.9608147780047815, 0.8790135383605957, -1.3906362692462366, -1.64390737215151, 0]
a3 = [5.416758060455322, -1.6744526068316858, 0.5214529037475586, -1.1861613432513636, -1.7241657415973108, -0.4453681151019495]

Pick_UP_Mediate = [5.693994998931885, -2.591994110737936, 1.884692668914795, -2.5459960142718714, -1.5736706892596644, 0.007104023825377226]

Place_Look = [6.242576599121094, -2.5770681540118616, 2.1108312606811523, -2.6321876684771937, -1.5052736441241663, -0.00026351610292607575]

Mat_to_Place_mediate = np.array([[ 0.06108396,-0.99805799,-0.01220661,0.14077532],\
                                 [ 0.0065976 ,-0.01182546,0.99990831 ,0.21332877],\
                                 [-0.99811083,-0.0611589 ,0.00586244 ,0.06646732],\
                                 [ 0.        , 0.        ,0.         ,1.        ]], dtype = np.float64)

Mat_to_Place_final = np.array([[0.01425125,-0.99986404,-0.00829426,0.1581478],\
                               [-0.00789913,-0.00840742,0.99993346,0.05041314],\
                               [-0.99986724,-0.01418479,-0.00801787,0.11220947],\
                               [ 0.        ,0.       ,0.        ,1.        ]], dtype = np.float64)

temp2 =                  np.array([[ 9.96073936e-01,8.68733127e-02,-1.70217702e-02,0.362],\
                               [-8.68777357e-02,9.96218865e-01,4.80845341e-04,-0.028],\
                               [ 1.69991812e-02,9.99855337e-04,9.99855004e-01,0.676],\
                               [ 0.            ,0.            ,0.            ,1.    ]], dtype = np.float64)

Place_mediate = [5.931669235229492, -1.5898664633380335, 0.9669747352600098, -2.5520828405963343, -1.3523181120501917, 0.005929944571107626]
Place_Final = [5.811901092529297, -1.8629072348224085, 1.6586275100708008, -2.955020252858297, -1.1851895491229456, 0.008038419298827648]

class FSM(object):
    """docstring for FSM"""
    def __init__(self):


        # TF listener
        self.listener = tf.TransformListener()

        # ROS publisher & subscriber & service
        # self.pub_target_pose = rospy.Publisher('target_pose', PoseStamped, queue_size=1)

        self.pick_srv = rospy.Service("pick_process", Empty, self.pick_process_cb)
        self.place_srv = rospy.Service("place_process", Empty, self.place_process_cb)
        self.home_srv = rospy.Service("home", Empty, self.home_cb)


    def place_process_cb(self, req):

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Place_Look[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        trans = 0; rot = 0
        try:
            (trans,rot) = self.listener.lookupTransform('/base_link', '/tag_324', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        mat_4x4 = np.eye(4, dtype = np.float64)
        target_mat = R.from_quat(rot).as_dcm()
        mat_4x4[:3,:3] = target_mat
        for i in range(3):
            mat_4x4[i,3] = trans[i]

        mat_4x4_med = np.dot(mat_4x4, Mat_to_Place_mediate)
        source_q = R.from_dcm(mat_4x4_med[:3,:3]).as_quat()

        msg = Pose()
        msg.position.x = mat_4x4_med[0,3] + 0.15
        msg.position.y = mat_4x4_med[1,3]
        msg.position.z = mat_4x4_med[2,3]
        msg.orientation.w = source_q[3]
        msg.orientation.x = source_q[0]
        msg.orientation.y = source_q[1]
        msg.orientation.z = source_q[2]

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_pose')
        try:
            ur5_pose_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_pose', target_pose)
            req = target_poseRequest()
            req.target_pose = msg
            req.factor = 0.5
            resp1 = ur5_pose_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        ####### print np.dot(np.linalg.inv(mat_4x4), temp2)

        mat_4x4_final = np.dot(mat_4x4, Mat_to_Place_final)
        source_q = R.from_dcm(mat_4x4_final[:3,:3]).as_quat()
        msg = Pose()
        msg.position.x = mat_4x4_final[0,3] + 0.15
        msg.position.y = mat_4x4_final[1,3]
        msg.position.z = mat_4x4_final[2,3]
        msg.orientation.w = source_q[3]
        msg.orientation.x = source_q[0]
        msg.orientation.y = source_q[1]
        msg.orientation.z = source_q[2]


        rospy.wait_for_service('/ur5_control_server/ur_control/goto_pose')
        try:
            ur5_pose_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_pose', target_pose)
            req = target_poseRequest()
            req.target_pose = msg
            req.factor = 0.5
            resp1 = ur5_pose_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        rospy.wait_for_service('/gripper_control/open')
        try:
            gripper_open_ser = rospy.ServiceProxy('/gripper_control/open', Empty)
            req = EmptyRequest()
            resp1 = gripper_open_ser(req)
            rospy.sleep(1)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e      

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Place_Look[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def home_cb(self, req):

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Prepare_To_Pick[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def pick_process_cb(self, req):
        # Update FSM state

        rospy.wait_for_service('/gripper_control/initial')
        try:
            gripper_close_ser = rospy.ServiceProxy('/gripper_control/initial', Empty)
            req = EmptyRequest()
            resp1 = gripper_close_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e   


        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = JOINTS_PICKING_HOME[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = a1[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = a2[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = a3[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Prepare_To_Pick[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print ('obj pose')
        rospy.wait_for_service('/object_pose_node')
        res_pose = object_onlyResponse()
        print ('start obj')
        # while True:
        try:
            pose_ser = rospy.ServiceProxy('/object_pose_node', object_only)
            req = object_onlyRequest()
            res_pose = pose_ser(req)
            print("End obj req")
            # if res_pose.ob_list[0].pose.position.x > 0.01 and res_pose.ob_list[0].pose.position.y > 0.01 and res_pose.ob_list[0].pose.position.z > 0.01:
            #     break

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print res_pose


        rospy.wait_for_service('/ur5_control_server/ur_control/goto_pose')
        try:
            ur5_pose_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_pose', target_pose)
            req = target_poseRequest()
            msg = Pose()

            if res_pose.count > 0:
                msg = res_pose.ob_list[0].pose
            else:
                rospy.loginfo("No Object Detected!")

            req.target_pose = msg
            req.factor = 0.5
            resp1 = ur5_pose_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/gripper_control/close')
        try:
            gripper_close_ser = rospy.ServiceProxy('/gripper_control/close', Empty)
            req = EmptyRequest()
            resp1 = gripper_close_ser(req)
            rospy.sleep(1.5)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        


        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Pick_UP_Mediate[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Pick_UP_Final[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # rospy.wait_for_service('/gripper_control/open')
        # try:
        #     gripper_open_ser = rospy.ServiceProxy('/gripper_control/open', Empty)
        #     req = EmptyRequest()
        #     resp1 = gripper_open_ser(req)
        #     rospy.sleep(0.5)
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e      

    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('system_fsm', anonymous=False)
    node = FSM()
    rospy.on_shutdown(node.shutdown_cb)
    rospy.spin()
