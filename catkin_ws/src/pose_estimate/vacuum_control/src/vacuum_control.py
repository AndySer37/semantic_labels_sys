#!/usr/bin/env python

import roslib
import rospy
from text_msgs.srv import *
from text_msgs.msg import *
import serial
import time
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse, EmptyRequest

class vacuum_control(object):
    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyACM0')
        baud = rospy.get_param('~baud', 115200)
        try:
            self.serial = serial.Serial(port=port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=2,rtscts=True,dsrdtr=True)
            self.serial.isOpen()   
            self.res = self.serial.readall()
            self.gripper_in = rospy.Service("~off", Empty, self.in_ser)
            self.gripper_out = rospy.Service("~on", Empty, self.out_ser)
            self.suck = rospy.Service("~suck", Empty, self.suck_ser)
            self.blow = rospy.Service("~blow", Empty, self.blow_ser)
            self.normal = rospy.Service("~normal", Empty, self.narmal_ser)
            self.weak_blow = rospy.Service("~weak_blow", Empty, self.weak_blow_ser) 
            ### Conveyor 
            self.Conveyor_start = rospy.Service("~conveyor_start", Empty, self.conveyor_start)

            rospy.loginfo("Vacuum Node Ready for port: {}, baudrate: {}.".format(port, baud))
        except (rospy.ServiceException, rospy.ROSException, serial.SerialException), e:
            rospy.logerr("Could not connect to vacuum system.")
            rospy.logerr(e)
        
        
    def in_ser(self, req):

        rospy.loginfo("Gripper status: 2-finger!")
        self.serial.write('p0')
        return EmptyResponse()

    def out_ser(self, req):

        rospy.loginfo("Gripper status: vacuum!")
        self.serial.write('p1')
        return EmptyResponse()

    def suck_ser(self, req):

        rospy.loginfo("Vacuun status: suck!")
        self.serial.write('v2')
        return EmptyResponse()

    def blow_ser(self, req):

        rospy.loginfo("Vacuun status: blow!")
        self.serial.write('v1')
        return EmptyResponse()

    def narmal_ser(self, req):

        rospy.loginfo("Vacuun status: narmal!")
        self.serial.write('v0')
        return EmptyResponse()

    def weak_blow_ser(self, req):

        rospy.loginfo("Vacuun status: weak blow")
        self.serial.write('v3')
        return EmptyResponse()

    def conveyor_start(self, req):
        self.serial.write('c1')
        rospy.loginfo("Conveyor status: Start")
        rospy.sleep(0.5)
        self.serial.write('c0')
        return EmptyResponse()

    def onShutdown(self):
        rospy.loginfo("Vacuum Control Node Shutdown.")  

if __name__ == '__main__': 
    rospy.init_node('vacuum_control',anonymous=False)
    vacuum_control = vacuum_control()
    rospy.on_shutdown(vacuum_control.onShutdown)
    rospy.spin()
