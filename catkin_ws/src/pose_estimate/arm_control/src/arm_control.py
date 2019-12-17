#!/usr/bin/env python

import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *
from arm_operation.srv import *
from arm_operation.msg import *
from std_srvs.srv import Trigger, TriggerResponse,TriggerRequest, Empty, EmptyResponse, EmptyRequest

from pyquaternion import Quaternion 
from converter import rpy_to_rot, rot_to_rpy
from scipy.spatial.transform import Rotation as R

Joint_Server = "/ur5_control_server/ur_control/goto_joint_pose"
Pose_Server = "/ur5_control_server/ur_control/goto_pose"
Straight_Server = "/ur5_control_server/ur_control/go_straight"

## Cam on Hand
# Home = [5.967301845550537, -1.7598593870746058, 1.4289522171020508, -1.2384098211871546, -1.4981196562396448, -0.08584672609438115]

## Vacuum Srv
Suck_srv = "/vacuum_control/suck"
Normal_srv = "/vacuum_control/normal"
Release_srv = "/vacuum_control/weak_blow"
Gripper_state = "/vacuum_control/off"
Vacuum_state = "/vacuum_control/on"

## Static Cam
Home = [4.781722545623779, -1.6464203039752405, 1.8191642761230469, -1.7565234343158167, -1.5184429327594202, 0.012554224580526352]
Flip_down = [6.197889804840088, -1.2037904898272913, 2.085622787475586, -1.858868424092428, -2.9599974791156214, 0.5323190689086914]
Flip_up = [6.190801620483398, -1.7303056875811976, 2.084195137023926, -1.858269993458883, -2.95280367532839, 0.5297435522079468]


class arm_control(object):
    def __init__(self):
        self.mani_req = manipulationRequest()

        self.arm_move_srv = rospy.Service("~move_to", manipulation, self.srv_move)
        self.arm_home_srv = rospy.Service("~home", Trigger, self.srv_home)
        self.flip_srv = rospy.Service("~flip", Trigger, self.srv_flip)
        self.suck_process_srv = rospy.Service("~suck_process", Trigger, self.srv_suck)

    def srv_move(self, req_mani):

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_pose')
        try:
            ur5_pose_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_pose', target_pose)
            req = target_poseRequest()
            req.target_pose = req_mani.pose
            req.factor = 0.5
            resp1 = ur5_pose_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def srv_home(self, req):

        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = Home[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.loginfo("No object. Finish the task")
        return TriggerResponse(success=True, message="Request accepted.")

    def srv_flip(self, req):
        self.joint_func(Flip_up)
        rospy.sleep(1.0)
        self.joint_func(Flip_down)
        rospy.sleep(1.0)
        self.suck_release()
        self.joint_func(Flip_up)
        return TriggerResponse(success=True, message="Request accepted.")

    def srv_suck(self, req):

        self.suck_func()
        return TriggerResponse(success=True, message="Request accepted.")


    def joint_func(self, joint):
        rospy.wait_for_service('/ur5_control_server/ur_control/goto_joint_pose')
        try:
            ur5_joint_ser = rospy.ServiceProxy('/ur5_control_server/ur_control/goto_joint_pose', joint_pose)
            req = joint_poseRequest()
            msg = joint_value()
            for i in range(6):
                msg.joint_value[i] = joint[i]
            req.joints.append(msg)
            req.factor = 0.5
            resp1 = ur5_joint_ser(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def suck_release(self):

        rospy.wait_for_service(Release_srv, timeout=10)
        release_obj = rospy.ServiceProxy(Release_srv, Empty)
        req = EmptyRequest()
        release_obj(req)
        rospy.sleep(0.5)
        rospy.wait_for_service(Gripper_state, timeout=10)
        two_finger_srv = rospy.ServiceProxy(Gripper_state, Empty)
        two_finger_srv(req)
        rospy.sleep(0.5)
        rospy.wait_for_service(Normal_srv, timeout=10)
        normal = rospy.ServiceProxy(Normal_srv, Empty)
        normal(req)
        rospy.sleep(0.5)

    def suck_func(self):
        rospy.wait_for_service(Vacuum_state, timeout=10)
        vacuum_on = rospy.ServiceProxy(Vacuum_state, Empty)
        req = EmptyRequest()
        vacuum_on(req)
        rospy.sleep(0.5)
        rospy.wait_for_service(Suck_srv, timeout=10)
        suck = rospy.ServiceProxy(Suck_srv, Empty)
        suck(req)
        rospy.sleep(0.5)

    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('arm_control', anonymous=False)
    node = arm_control()
    rospy.on_shutdown(node.shutdown_cb)
    rospy.spin()

