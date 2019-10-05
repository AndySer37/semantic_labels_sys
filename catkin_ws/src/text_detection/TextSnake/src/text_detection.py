#!/usr/bin/env python

import numpy as np
import cv2
import roslib
import rospy
import tf
import struct
import math
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
import os 
import message_filters
from text_msgs.msg import text_detection_msg, text_detection_array, int_arr
from text_msgs.srv import *

# from dataset.deploy import DeployDataset
from network.textnet import TextNet
from util.detection import TextDetector
from util.augmentation import BaseTransform
from util.config import config as cfg, update_config, print_config
from util.option import BaseOptions
from util.visualize import visualize_detection
from util.misc import to_device, mkdirs, rescale_result

RECOG_SRV = "/text_recognize/text_recognize_server"

class text_detection(object):
	def __init__(self):
		self.switch = False
		r = rospkg.RosPack()
		self.path = r.get_path('TextSnake')
		self.prob_threshold = 0.90
		self.cv_bridge = CvBridge() 
		self.means = (0.485, 0.456, 0.406)
		self.stds = (0.229, 0.224, 0.225)

		self.objects = []
		self.network = TextNet(is_training=False, backbone='vgg')
		self.is_compressed = False

		self.cuda_use = torch.cuda.is_available()

		if self.cuda_use:
			self.network = self.network.cuda()

		model_name = "textsnake_vgg_0.pth"
		self.network.load_model(os.path.join(self.path, "weights/", model_name))

		self.detector = TextDetector(self.network, tr_thresh=0.6, tcl_thresh=0.4)
		self.network.eval()
		#### Publisher
		self.image_pub = rospy.Publisher("~predict_img", Image, queue_size = 1)
		self.img_bbox_pub = rospy.Publisher("~predict_bbox", Image, queue_size = 1)
		self.text_detection_pub = rospy.Publisher("/text_detection_array", text_detection_array, queue_size = 1)
		### service
		self.predict_switch_ser = rospy.Service("~predict_switch_server", predict_switch, self.switch_callback)
		self.predict_ser = rospy.Service("~text_detection", text_detection_srv, self.srv_callback)
		### msg filter 
		image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
		depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
		ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
		ts.registerCallback(self.callback)
		print "============ Ready ============"

	def srv_callback(self, req):
		text_array = text_detection_array()

		resp = text_detection_srvResponse()
		img_msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=None)
		resp.depth = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image, timeout=None)
		resp.image = img_msg
		try:
			if self.is_compressed:
				np_arr = np.fromstring(img_msg.data, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			resp.status = e
			print(e)
		
		# (rows, cols, channels) = cv_image.shape
		# self.width = cols
		# self.height = rows
		
		predict_img, contours = self.predict(cv_image)
		img_bbox = cv_image.copy()

		text_array = text_detection_array()
		text_array.image = img_msg
		text_array.depth = resp.depth
		for _cont in contours:
			text_bb = text_detection_msg()
			for p in _cont:
				int_array = int_arr()
				int_array.point.append(p[0])
				int_array.point.append(p[1])
				text_bb.contour.append(int_array)
			# cv2.drawContours(cv_image, [_cont], -1, (0, 255, 0), 2)
			text_bb.box.xmin = min(_cont[:,0])
			text_bb.box.xmax = max(_cont[:,0])
			text_bb.box.ymin = min(_cont[:,1])
			text_bb.box.ymax = max(_cont[:,1])
			text_array.text_array.append(text_bb)
			cv2.rectangle(img_bbox, (text_bb.box.xmin, text_bb.box.ymin),(text_bb.box.xmax, text_bb.box.ymax), (255, 0, 0), 3)
		text_array.bb_count = len(text_array.text_array)
		# self.text_detection_pub.publish(text_array)

		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
			self.img_bbox_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_bbox, "bgr8"))
		except CvBridgeError as e:
			resp.state = e
			print(e)

		recog_req = text_recognize_srvRequest()
		try:
			rospy.wait_for_service(RECOG_SRV, timeout=10)
			recog_req.data = text_array
			recognition_srv = rospy.ServiceProxy(RECOG_SRV, text_recognize_srv)
			recog_resp = recognition_srv(recog_req)
		except (rospy.ServiceException, rospy.ROSException), e:
			resp.state = e
		resp.mask = recog_resp.mask
		return resp


	def callback(self, img_msg, depth):
		if not self.switch:
			return
		try:
			if self.is_compressed:
				np_arr = np.fromstring(img_msg.data, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		# (rows, cols, channels) = cv_image.shape
		# self.width = cols
		# self.height = rows
		
		predict_img, contours = self.predict(cv_image)
		img_bbox = cv_image.copy()

		text_array = text_detection_array()
		text_array.image = img_msg
		text_array.depth = depth
		for _cont in contours:
			text_bb = text_detection_msg()
			for p in _cont:
				int_array = int_arr()
				int_array.point.append(p[0])
				int_array.point.append(p[1])
				text_bb.contour.append(int_array)
			# cv2.drawContours(cv_image, [_cont], -1, (0, 255, 0), 2)
			text_bb.box.xmin = min(_cont[:,0])
			text_bb.box.xmax = max(_cont[:,0])
			text_bb.box.ymin = min(_cont[:,1])
			text_bb.box.ymax = max(_cont[:,1])
			text_array.text_array.append(text_bb)
			cv2.rectangle(img_bbox, (text_bb.box.xmin, text_bb.box.ymin),(text_bb.box.xmax, text_bb.box.ymax), (255, 0, 0), 3)
		text_array.bb_count = len(text_array.text_array)
		self.text_detection_pub.publish(text_array)

		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
			self.img_bbox_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_bbox, "bgr8"))
		except CvBridgeError as e:
			print(e)

	def predict(self, img):
		# # Preprocessing
		(rows, cols, channels) = img.shape
		image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

		torch.cuda.synchronize()
		start = time.time()

		x = image.astype(np.float32)
		x = (x / 255 - self.means) / self.stds
		x = x.astype(np.float32)
		x = x[:, :, ::-1].copy()
		x = torch.from_numpy(x).permute(2, 0, 1)
		x = Variable(x.unsqueeze(0)) 
		if self.cuda_use:
			x = x.cuda()
		contours, output = self.detector.detect(x)

		torch.cuda.synchronize()
		end = time.time()
		print "Text Detection Time : {}".format(end - start)

		image, contours = rescale_result(image, contours, rows, cols)
		img_viz = visualize_detection(image, contours)

		return img_viz, contours

	def switch_callback(self, req):
		resp = predict_switchResponse()
		self.switch = req.data
		s = "True" if req.data else "False"
		resp.result = "Switch turn to {}".format(req.data)
		return resp

	def onShutdown(self):
		rospy.loginfo("Shutdown.")	
	

if __name__ == '__main__': 
	rospy.init_node('text_detection',anonymous=False)
	text_detection = text_detection()
	rospy.on_shutdown(text_detection.onShutdown)
	rospy.spin()
