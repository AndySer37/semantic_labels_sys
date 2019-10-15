#!/usr/bin/env python

import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *
from arm_operation.srv import *
from arm_operation.msg import *
from std_srvs.srv import Trigger, TriggerResponse, Empty

Joint_Server = "/ur5_control_server/ur_control/goto_joint_pose"
Pose_Server = "/ur5_control_server/ur_control/goto_pose"
Straight_Server = "/ur5_control_server/ur_control/go_straight"

# /ur5_control_server/ur_control/get_robot_state
# /ur5_control_server/ur_control/go_straight
# /ur5_control_server/ur_control/goto_joint_pose
# /ur5_control_server/ur_control/goto_pose
# /ur5_control_server/ur_control/stop_program
# /ur5_control_server/ur_control/unlock_protective

class arm_control(object):
    def __init__(self):
        self.mani_req = manipulationRequest()

        self.arm_move_srv = rospy.Service("~/move_to", Trigger, self.srv_callback)
        self.arm_home_srv = rospy.Service("~/home", Trigger, self.srv_home)

    def srv_callback(self, req):

        print pose_res.count 
        rospy.loginfo("No object. Finish the task")
        return "finish"

    def srv_home(self, req):

        print pose_res.count 
        rospy.loginfo("No object. Finish the task")
        return "finish"



def main():
    rospy.init_node('arm_control')
    rospy.wait_for_service(Joint_Server)
    rospy.wait_for_service(Pose_Server)
    rospy.wait_for_service(Straight_Server)
    rospy.spin()

if __name__ == '__main__':
    main()
