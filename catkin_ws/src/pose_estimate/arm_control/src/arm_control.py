#!/usr/bin/env python

import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *

mani_req = manipulationRequest()

class arm_control(object):
    def __init__(self):
        self.arm_move = rospy.Service("", , self.switch_callback)

    def srv_callback(self, userdata):
        global mani_req
        mani_req =  manipulationRequest()

        per_res = self.per_client() #Call service
        pose_req = pose_stateRequest()
        pose_req.image = per_res.image
        pose_req.depth = per_res.depth 
        pose_req.mask = per_res.mask
        pose_res = self.pose_client(pose_req)

        print pose_res.count 
        rospy.loginfo("No object. Finish the task")
        return "finish"




def main():
    rospy.init_node('arm_control')
    rospy.wait_for_service(Detect_SRV)
    rospy.wait_for_service(Pose_SRV)
    rospy.spin()

if __name__ == '__main__':
    main()
