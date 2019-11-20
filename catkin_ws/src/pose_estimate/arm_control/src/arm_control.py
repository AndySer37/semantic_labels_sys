#!/usr/bin/env python

import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *
from arm_operation.srv import *
from arm_operation.msg import *
from std_srvs.srv import Trigger, TriggerResponse, Empty
from pyquaternion import Quaternion 
from converter import rpy_to_rot, rot_to_rpy
from scipy.spatial.transform import Rotation as R

Joint_Server = "/ur5_control_server/ur_control/goto_joint_pose"
Pose_Server = "/ur5_control_server/ur_control/goto_pose"
Straight_Server = "/ur5_control_server/ur_control/go_straight"

Home = [5.967301845550537, -1.7598593870746058, 1.4289522171020508, -1.2384098211871546, -1.4981196562396448, -0.08584672609438115]

class arm_control(object):
    def __init__(self):
        self.mani_req = manipulationRequest()

        self.arm_move_srv = rospy.Service("~move_to", Trigger, self.srv_callback)
        self.arm_home_srv = rospy.Service("~home", Trigger, self.srv_home)

    def srv_callback(self, req):

        print pose_res.count 
        rospy.loginfo("No object. Finish the task")
        return "finish"

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

    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")


if __name__ == '__main__':
    rospy.init_node('arm_control', anonymous=False)
    node = arm_control()
    rospy.on_shutdown(node.shutdown_cb)
    rospy.spin()

