#!/usr/bin/env python

import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input, Robotiq2FGripper_robot_output
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse, EmptyRequest


class gripper_control(object):
    def __init__(self):

        self.gripper_pub = rospy.Publisher("/Robotiq2FGripperRobotOutput", Robotiq2FGripper_robot_output, queue_size = 1)   
        self.close_ser = rospy.Service("~close", Empty, self.close_ser)
        self.open_ser = rospy.Service("~open", Empty, self.open_ser)
        self.initial_ser = rospy.Service("~initial", Empty, self.initial_ser)
        self.reset_ser = rospy.Service("~reset", Empty, self.reset_ser)
        self.get_state_ser = rospy.Service("~get_state", Empty, self.get_state_ser)

    def close_ser(self, req):

        rospy.loginfo("Close gripper!")
        msg = Robotiq2FGripper_robot_output()
        msg.rACT = 1; msg.rGTO = 1; msg.rPR = 255; msg.rSP = 200; msg.rFR = 255
        self.gripper_pub.publish(msg)
        return EmptyResponse()

    def open_ser(self, req):

        rospy.loginfo("Open gripper!")
        msg = Robotiq2FGripper_robot_output()
        msg.rACT = 1; msg.rGTO = 1; msg.rPR = 0; msg.rSP = 200; msg.rFR = 100
        self.gripper_pub.publish(msg)
        return EmptyResponse()

    def initial_ser(self, req):

        rospy.wait_for_service('~reset')
        try:
            reset = rospy.ServiceProxy('~reset', Empty)
            reset()
        except:
            print "Reset error"
        rospy.sleep(1)
        rospy.loginfo("Initial gripper!")
        msg = Robotiq2FGripper_robot_output()       
        msg.rACT = 1; msg.rGTO = 1; msg.rSP = 200; msg.rFR = 100
        self.gripper_pub.publish(msg)
        return EmptyResponse()

    def reset_ser(self, req):

        rospy.loginfo("Reset gripper!")
        msg = Robotiq2FGripper_robot_output()
        self.gripper_pub.publish(msg)
        return EmptyResponse()

    def get_state_ser(self, req):

        rospy.loginfo("Getting state")
        msg = rospy.wait_for_message('/Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input)
        print msg
        return EmptyResponse()

    def onShutdown(self):
        rospy.loginfo("Shutdown.")  

if __name__ == '__main__': 
    rospy.init_node('gripper_control',anonymous=False)
    gripper_control = gripper_control()
    rospy.on_shutdown(gripper_control.onShutdown)
    rospy.spin()
