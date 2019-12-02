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
from rotate_input import rotate_cv, rotate_back, rotate_back_change_h_w

RECOG_SRV = "/text_recognize/text_recognize_server"

class text_detection(object):
	def __init__(self):
		self.switch = False
		r = rospkg.RosPack()
		self.path = r.get_path('textsnake')
		self.commodity_list = []
		self.read_commodity(r.get_path('text_msgs') + "/config/commodity_list.txt")
		self.prob_threshold = 0.90
		self.cv_bridge = CvBridge() 
		self.means = (0.485, 0.456, 0.406)
		self.stds = (0.229, 0.224, 0.225)

		self.saver = False

		self.color_map = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,255,255)] # 0 90 180 270 noise

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
		self.predict_img_pub = rospy.Publisher("/prediction_img", Image, queue_size = 1)
		self.predict_mask_pub = rospy.Publisher("/prediction_mask", Image, queue_size = 1)
		self.text_detection_pub = rospy.Publisher("/text_detection_array", text_detection_array, queue_size = 1)
		### service
		self.predict_switch_ser = rospy.Service("~predict_switch_server", predict_switch, self.switch_callback)
		self.predict_ser = rospy.Service("~text_detection", text_detection_srv, self.srv_callback)
		### msg filter 
		image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
		depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
		ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
		ts.registerCallback(self.callback)
		self.saver_count = 0
		if self.saver:
			self.p_img = os.path.join(self.path, "saver", "img")
			if not os.path.exists(self.p_img):
				os.makedirs(self.p_img)
			self.p_depth = os.path.join(self.path, "saver", "depth")
			if not os.path.exists(self.p_depth):
				os.makedirs(self.p_depth)
			self.p_mask = os.path.join(self.path, "saver", "mask")
			if not os.path.exists(self.p_mask):
				os.makedirs(self.p_mask)
			self.p_result = os.path.join(self.path, "saver", "result")
			if not os.path.exists(self.p_result):
				os.makedirs(self.p_result)

		print "============ Ready ============"

	def read_commodity(self, path):

		for line in open(path, "r"):
			line = line.rstrip('\n')
			self.commodity_list.append(line)
		print "Node (text_detection): Finish reading list"

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
		(rows, cols, channels) = cv_image.shape	
		rows = int(np.ceil(rows/32.)*32)
		cols = int(np.ceil(cols/32.)*32)
		cv_image1 = np.zeros((rows, cols, channels),dtype = np.uint8)
		cv_image1[:cv_image.shape[0],:cv_image.shape[1],:cv_image.shape[2]] = cv_image[:,:,:]
		cv_image = cv_image1.copy()

		mask = np.zeros([cv_image.shape[0], cv_image.shape[1]], dtype = np.uint8)
		img_list_0_90_180_270 = rotate_cv(cv_image)

		for i in range(4):

			predict_img, contours = self.predict(img_list_0_90_180_270[i])
			img_bbox = img_list_0_90_180_270[i].copy()

			text_array = text_detection_array()
			text_array.image = self.cv_bridge.cv2_to_imgmsg(img_list_0_90_180_270[i], "bgr8")
			text_array.depth = resp.depth
			for _cont in contours:
				text_bb = text_detection_msg()
				for p in _cont:
					int_array = int_arr()
					int_array.point.append(p[0])
					int_array.point.append(p[1])
					text_bb.contour.append(int_array)
				cv2.drawContours(predict_img, [_cont], -1, self.color_map[i], 3)
				text_bb.box.xmin = min(_cont[:,0])
				text_bb.box.xmax = max(_cont[:,0])
				text_bb.box.ymin = min(_cont[:,1])
				text_bb.box.ymax = max(_cont[:,1])
				text_array.text_array.append(text_bb)
				cv2.rectangle(img_bbox, (text_bb.box.xmin, text_bb.box.ymin),(text_bb.box.xmax, text_bb.box.ymax), self.color_map[i], 3)
			text_array.bb_count = len(text_array.text_array)
			# self.text_detection_pub.publish(text_array)
			
			recog_req = text_recognize_srvRequest()
			recog_resp = text_recognize_srvResponse()
			try:
				rospy.wait_for_service(RECOG_SRV, timeout=10)
				recog_req.data = text_array
				recog_req.direct = i
				recognition_srv = rospy.ServiceProxy(RECOG_SRV, text_recognize_srv)
				recog_resp = recognition_srv(recog_req)
			except (rospy.ServiceException, rospy.ROSException), e:
				resp.state = e

			recog_mask = self.cv_bridge.imgmsg_to_cv2(recog_resp.mask, "8UC1")

			if i == 0:
				pass
			elif i == 1:
				recog_mask = rotate_back_change_h_w(recog_mask, angle = -90)
				predict_img = rotate_back_change_h_w(predict_img, angle = -90)
				img_bbox = rotate_back_change_h_w(img_bbox, angle = -90)
			elif i == 2:
				recog_mask = rotate_back(recog_mask, angle = -180)
				predict_img = rotate_back(predict_img, angle = -180)
				img_bbox = rotate_back(img_bbox, angle = -180)
			else:
				recog_mask = rotate_back_change_h_w(recog_mask, angle = -270)
				predict_img = rotate_back_change_h_w(predict_img, angle = -270)
				img_bbox = rotate_back_change_h_w(img_bbox, angle = -270)

			mask[recog_mask != 0] = recog_mask[recog_mask != 0]

			try:
				self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
				self.img_bbox_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_bbox, "bgr8"))
			except CvBridgeError as e:
				resp.state = e
				print(e)

		## publish visualization
		self.img_show(mask, cv_image)
		resp.mask = self.cv_bridge.cv2_to_imgmsg(mask, "8UC1")
		vis_mask = np.zeros([cv_image.shape[0], cv_image.shape[1]], dtype = np.uint8)
		vis_mask[mask != 0] = 255 - mask[mask != 0]
		if self.saver:
			self.save_func(cv_image1, vis_mask, self.cv_bridge.imgmsg_to_cv2(resp.depth, "16UC1"), cv_image)
		## srv end
		self.predict_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		self.predict_mask_pub.publish(self.cv_bridge.cv2_to_imgmsg(vis_mask, "8UC1"))
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
		
		(rows, cols, channels) = cv_image.shape
		# self.width = cols
		# self.height = rows
		rows = int(np.ceil(rows/32.)*32)
		cols = int(np.ceil(cols/32.)*32)
		cv_image1 = np.zeros((rows, cols, channels),dtype = np.uint8)
		cv_image1[:cv_image.shape[0],:cv_image.shape[1],:cv_image.shape[2]] = cv_image[:,:,:]
		cv_image = cv_image1.copy()
		# cv_image = cv2.resize(cv_image, (1280,736))
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

	def img_show(self, mask, img):
		obj_num = len(self.commodity_list)
		_list = np.unique(mask)
		print _list
		for i in _list:			# self.commodity_list
			obj = i % obj_num
			direct = int(i / obj_num)
			if obj != 0:
				mask_i = mask.copy()
				mask_i[mask_i != i] = 0
				_, contours, hierarchy = cv2.findContours(mask_i, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

				for j in range(len(contours)):
					cnt = contours[j]
					area = cv2.contourArea(cnt) 
					if area > 1000:
						cv2.drawContours(img, [cnt], -1, self.color_map[direct], 3)
						x,y,w,h = cv2.boundingRect(cnt)
						word = self.commodity_list[obj] + " " + str(direct*90)
						cv2.putText(img, word, (x, y), 0, 1, (0, 255, 255),2)


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

	def save_func(self, img, mask, depth, result):
		cv2.imwrite("{}/{}.png".format(self.p_img, self.saver_count), img)
		cv2.imwrite("{}/{}.png".format(self.p_mask, self.saver_count), mask)
		cv2.imwrite("{}/{}.png".format(self.p_depth, self.saver_count), depth)
		cv2.imwrite("{}/{}.png".format(self.p_result, self.saver_count), result)

		self.saver_count += 1

	def onShutdown(self):
		rospy.loginfo("Shutdown.")	
	

if __name__ == '__main__': 
	rospy.init_node('text_detection',anonymous=False)
	text_detection = text_detection()
	rospy.on_shutdown(text_detection.onShutdown)
	rospy.spin()
