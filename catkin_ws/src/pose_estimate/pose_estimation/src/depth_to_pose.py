#!/usr/bin/env python
import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
import csv
from std_msgs.msg import Header
from text_msgs.msg import *
from text_msgs.srv import *
from sensor_msgs.msg import Image, CameraInfo
from tf import TransformListener,TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import rospkg
from cv_bridge import CvBridge, CvBridgeError

Object_Pose_SRV = "/object_pose_node"
Brand_name_Pose_SRV = "/bn_pose_node"

class depth_to_pose():
	def __init__(self):
		r = rospkg.RosPack()
		self.ee_frame = "/ee_link"
		self.base_frame = "/base_link"
		self.cam_frame = "/camera_link"
		self.commodity_list = []
		self.read_commodity(r.get_path('text_msgs') + "/config/commodity_list.txt")
		self.node_name = rospy.get_name()
		self.header = Header()
		self.header.frame_id = self.cam_frame
		rospy.loginfo("[%s] Initializing " %(self.node_name))
		self.cv_bridge = CvBridge()


		frame_switch_srv = rospy.Service('~get_pose', pose_state, self.srv_callback)

	def srv_callback(self, req):
		resp = pose_stateResponse()
		upward_list = np.unique(self.cv_bridge.imgmsg_to_cv2(req.mask, "8UC1"))
		print len(upward_list)
		if len(upward_list) == 1:
			object_req = object_onlyRequest()
			try:
				rospy.wait_for_service(Object_Pose_SRV, timeout=10)
				object_req.image = req.image
				object_req.depth = req.depth
				recognition_srv = rospy.ServiceProxy(Object_Pose_SRV, object_only)
				recog_resp = recognition_srv(object_req)
				resp.count = recog_resp.count
				resp.ob_list = recog_resp.ob_list
			except (rospy.ServiceException, rospy.ROSException), e:
				resp.state = "error"


		else:
			bn_req = bn_pose_srvRequest()
			bn_req.count = len(upward_list) - 1
			for i in upward_list:
				if i != 0:
					bn_req.list.append(i)
			try:
				print bn_req.list
				rospy.wait_for_service(Brand_name_Pose_SRV, timeout=10)
				bn_req.image = req.image
				bn_req.depth = req.depth
				bn_req.mask = req.mask
				recognition_srv = rospy.ServiceProxy(Brand_name_Pose_SRV, bn_pose_srv)
				recog_resp = recognition_srv(bn_req)
				resp.count = recog_resp.count
				resp.ob_list = recog_resp.ob_list	
			except (rospy.ServiceException, rospy.ROSException), e:
				resp.state = "error"


		return resp

	def read_commodity(self, path):

		for line in open(path, "r"):
			line = line.rstrip('\n')
			self.commodity_list.append(line)
		print "Finish reading list"

	def onShutdown(self):

		rospy.loginfo("Shutdown and save file!!!")

if __name__ == '__main__':
	rospy.init_node('depth_to_pose')
	foo = depth_to_pose()
	rospy.on_shutdown(foo.onShutdown)
	rospy.spin()



