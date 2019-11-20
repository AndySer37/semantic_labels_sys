#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from text_msgs.srv import *
from text_msgs.msg import *
from std_srvs.srv import Trigger, TriggerResponse, Empty

Initial_gripper = '/gripper_control/initial'
Detect_SRV = "/text_detection/text_detection"
# Pose_SRV = "/depth_to_pose/get_pose"
Object_Pose_SRV = "/object_pose_node"
Brand_name_Pose_SRV = "/bn_pose_node"

Initial = 0
Perception_bn = 1
pick_bn = 2
Perception_obj = 3
pick_obj = 4
flip = 5





class FSM():
    def __init__(self):
        self.state = -99
        self.mani_req = manipulationRequest()
        self.start = rospy.Service("~start", Trigger, self.srv_start)

    def srv_start(self, req):
        self.state = 0
        return TriggerResponse(success=True, message="Request accepted.")
    def process(self):
        if self.state == 0:
            rospy.wait_for_service(Initial_gripper)
            try:
                gripper_close_ser = rospy.ServiceProxy(Initial_gripper, Empty)
                req = EmptyRequest()
                resp1 = gripper_close_ser(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

            self.state = 1

        if self.state == 1:
            rospy.wait_for_service(Detect_SRV)
            try:
                gripper_close_ser = rospy.ServiceProxy(Detect_SRV, text_detection_srv)
                req = EmptyRequest()
                resp1 = gripper_close_ser(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 
            
            print  np.unique(self.cv_bridge.imgmsg_to_cv2(resp1.mask, "8UC1"))
            
    def shutdown_cb(self):
        rospy.loginfo("Node shutdown")

if __name__ == '__main__':
    rospy.init_node('fsm')
    foo = FSM()
    rospy.on_shutdown(foo.shutdown_cb)
    while not rospy.is_shutdown():
        foo.process()
        rospy.sleep(1)

    rospy.spin()