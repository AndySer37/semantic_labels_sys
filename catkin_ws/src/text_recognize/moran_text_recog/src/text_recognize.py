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

from PIL import Image as Im
import tools.utils as utils
import tools.dataset as dataset
from models.moran import MORAN
from collections import OrderedDict
class text_recognize(object):
	def __init__(self):
		r = rospkg.RosPack()
		self.path = r.get_path('moran_text_recog')
		self.prob_threshold = 0.90
		self.cv_bridge = CvBridge()
		self.commodity_list = []
		self.read_commodity(r.get_path('text_msgs') + "/config/commodity_list.txt")
		self.alphabet = '0:1:2:3:4:5:6:7:8:9:a:b:c:d:e:f:g:h:i:j:k:l:m:n:o:p:q:r:s:t:u:v:w:x:y:z:$' 

		self.means = (0.485, 0.456, 0.406)
		self.stds = (0.229, 0.224, 0.225)
		self.bbox_thres = 1500

		self.color_map = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,255,255)] # 0 90 180 270 noise

		self.objects = []
		self.is_compressed = False

		self.cuda_use = torch.cuda.is_available()

		if self.cuda_use:
			cuda_flag = True
			self.network = MORAN(1, len(self.alphabet.split(':')), 256, 32, 100, BidirDecoder=True, CUDA=cuda_flag)
			self.network = self.network.cuda()
		else:
			self.network = MORAN(1, len(self.alphabet.split(':')), 256, 32, 100, BidirDecoder=True, inputDataType='torch.FloatTensor', CUDA=cuda_flag)

		model_name = "moran.pth"
		print "Moran Model Parameters number: " + str(self.count_parameters(self.network))
		if self.cuda_use:
		    state_dict = torch.load(os.path.join(self.path, "weights/", model_name))
		else:
		    state_dict = torch.load(os.path.join(self.path, "weights/", model_name), map_location='cpu')
		MORAN_state_dict_rename = OrderedDict()
		for k, v in state_dict.items():
		    name = k.replace("module.", "") # remove `module.`
		    MORAN_state_dict_rename[name] = v
		self.network.load_state_dict(MORAN_state_dict_rename)
		self.converter = utils.strLabelConverterForAttention(self.alphabet, ':')
		self.transformer = dataset.resizeNormalize((100, 32))

		for p in self.network.parameters():
		    p.requires_grad = False
		self.network.eval()

		#### Publisher
		self.image_pub = rospy.Publisher("~predict_img", Image, queue_size = 1)
		self.mask = rospy.Publisher("~mask", Image, queue_size = 1)
		self.img_bbox_pub = rospy.Publisher("~predict_bbox", Image, queue_size = 1)
		#### Service
		self.predict_ser = rospy.Service("~text_recognize_server", text_recognize_srv, self.srv_callback)

		image_sub1 = rospy.Subscriber('/text_detection_array', text_detection_array, self.callback, queue_size = 1)
		### msg filter 
		# image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
		# depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
		# ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
		# ts.registerCallback(self.callback)
		print "============ Ready ============"


	def read_commodity(self, path):

		for line in open(path, "r"):
			line = line.rstrip('\n')
			self.commodity_list.append(line)
		print "Node (text_recognize): Finish reading list"

	def count_parameters(self, model):
		return sum(p.numel() for p in model.parameters() if p.requires_grad)

	def callback(self, msg):
		try:
			if self.is_compressed:
				np_arr = np.fromstring(msg.image, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(msg.image, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		predict_img, mask = self.predict(msg, cv_image)
		img_bbox = cv_image.copy()

		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
			self.img_bbox_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_bbox, "bgr8"))
			self.mask.publish(self.cv_bridge.cv2_to_imgmsg(mask, "8UC1"))
		except CvBridgeError as e:
			print(e)

	def srv_callback(self, req):
		resp = text_recognize_srvResponse()
		try:
			if self.is_compressed:
				np_arr = np.fromstring(req.data.image, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(req.data.image, "bgr8")
		except CvBridgeError as e:
			resp.state = e
			print(e)
		
		predict_img, mask = self.predict(req.data, cv_image, req.direct)
		img_bbox = cv_image.copy()

		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
			self.img_bbox_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_bbox, "bgr8"))
			resp.mask = self.cv_bridge.cv2_to_imgmsg(mask, "8UC1")
			self.mask.publish(self.cv_bridge.cv2_to_imgmsg(mask, "8UC1"))
		except CvBridgeError as e:
			resp.state = e
			print(e)

		return resp

	def predict(self, msg, img, rot=0):
		# # Preprocessing
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		(rows, cols, channels) = img.shape
		mask = np.zeros([rows, cols], dtype = np.uint8)

		for text_bb in msg.text_array:
			if (text_bb.box.ymax - text_bb.box.ymin) * (text_bb.box.xmax - text_bb.box.xmin) < self.bbox_thres:
				continue
			start = time.time()
			image = gray[text_bb.box.ymin:text_bb.box.ymax, text_bb.box.xmin:text_bb.box.xmax]

			image = Im.fromarray(image) 
			image = self.transformer(image)

			if self.cuda_use:
			    image = image.cuda()
			image = image.view(1, *image.size())
			image = Variable(image)
			text = torch.LongTensor(1 * 5)
			length = torch.IntTensor(1)
			text = Variable(text)
			length = Variable(length)

			max_iter = 20
			t, l = self.converter.encode('0'*max_iter)
			utils.loadData(text, t)
			utils.loadData(length, l)
			output = self.network(image, length, text, text, test=True, debug=True)

			preds, preds_reverse = output[0]
			demo = output[1]

			_, preds = preds.max(1)
			_, preds_reverse = preds_reverse.max(1)

			sim_preds = self.converter.decode(preds.data, length.data)
			sim_preds = sim_preds.strip().split('$')[0]
			sim_preds_reverse = self.converter.decode(preds_reverse.data, length.data)
			sim_preds_reverse = sim_preds_reverse.strip().split('$')[0]

			# print('\nResult:\n' + 'Left to Right: ' + sim_preds + '\nRight to Left: ' + sim_preds_reverse + '\n\n')
			print "Text Recognize Time : {}".format(time.time() - start)

			_cont = []
			for p in text_bb.contour:
				point = []
				point.append(p.point[0])
				point.append(p.point[1])
				_cont.append(point)
			_cont = np.array(_cont, np.int32)
			if sim_preds in self.commodity_list:
				cv2.rectangle(img, (text_bb.box.xmin, text_bb.box.ymin),(text_bb.box.xmax, text_bb.box.ymax), self.color_map[rot], 3)
				cv2.putText(img, sim_preds, (text_bb.box.xmin, text_bb.box.ymin), 0, 1, (0, 255, 255),3)
				cv2.fillConvexPoly(mask, _cont, self.commodity_list.index(sim_preds) + rot*len(self.commodity_list))
			else:
				correct, conf, _bool = self.conf_of_word(sim_preds)

				# print conf
				if _bool:
					cv2.putText(img, correct + "{:.2f}".format(conf), (text_bb.box.xmin, text_bb.box.ymin), 0, 1, (0, 255, 255),3)
					cv2.rectangle(img, (text_bb.box.xmin, text_bb.box.ymin),(text_bb.box.xmax, text_bb.box.ymax), (255, 255, 255), 2)
					cv2.fillConvexPoly(mask, _cont, self.commodity_list.index(correct) + rot*len(self.commodity_list))

				# else:
				# 	cv2.putText(img, sim_preds, (text_bb.box.xmin, text_bb.box.ymin), 0, 1, (0, 0, 0),3)
				# 	cv2.rectangle(img, (text_bb.box.xmin, text_bb.box.ymin),(text_bb.box.xmax, text_bb.box.ymax), (0, 0, 0), 2)					

		return img, mask

	def conf_of_word(self, target):
		### Edit distance
		total = np.zeros(len(self.commodity_list))
		for i in range(1, len(self.commodity_list)):
			size_x = len(self.commodity_list[i]) + 1
			size_y = len(target) + 1
			matrix = np.zeros ((size_x, size_y))
			for x in xrange(size_x):
				matrix [x, 0] = x
			for y in xrange(size_y):
				matrix [0, y] = y

			for x in xrange(1, size_x):
				for y in xrange(1, size_y):
					if self.commodity_list[i][x-1] == target[y-1]:
						matrix [x,y] = min(
							matrix[x-1, y] + 1,
							matrix[x-1, y-1],
							matrix[x, y-1] + 1
						)
					else:
						matrix [x,y] = min(
							matrix[x-1,y] + 1,
							matrix[x-1,y-1] + 1,
							matrix[x,y-1] + 1
						)
			# print (matrix)
			total[i] = (size_x - matrix[size_x-1, size_y-1]) / float(size_x)
		return self.commodity_list[np.argmax(total)], np.max(total), np.max(total) >= 0.66		


		### old method
		# total = np.zeros(len(self.commodity_list))
		# for i in range(1, len(self.commodity_list)):

		# 	# if self.commodity_list[i] != "raisins":
		# 	# 	continue

		# 	err = 0  ## error 
		# 	_len = len(self.commodity_list[i])
		# 	arr = -10 * np.ones(_len)
		# 	for j in range(len(target)):
		# 		index = self.commodity_list[i].find(target[j])
		# 		if index == -1:
		# 			err += 1
		# 		else:
		# 			upper = arr[index+1] if index != _len - 1 else -10
		# 			if arr[index] == -10 and upper == -10:
		# 				arr[index] = j
		# 			else:
		# 				index = self.commodity_list[i].find(target[j], index + 1)
		# 				while index != -1:
		# 					lower = arr[index-1] if index != 0 else -10
		# 					upper = arr[index+1] if index != _len - 1 else -10
		# 					if (arr[index] - lower) == 1 or (upper - arr[index]) == 1:
		# 						index = self.commodity_list[i].find(target[j], index + 1)
		# 					else:
		# 						arr[index] = j
		# 						break

		# 	score = 0   # score for word 
		# 	for j in range(_len - 1):
		# 		if arr[j+1] - arr[j] == 1:
		# 			score += 1
		# 	total[i] = float(score) / (_len + err - 1)
		# 	# print score, _len, err, arr


		# return self.commodity_list[np.argmax(total)], np.max(total), np.max(total) >= 0.5

	def onShutdown(self):
		rospy.loginfo("Shutdown.")	
	

if __name__ == '__main__': 
	rospy.init_node('text_recognize',anonymous=False)
	text_recognize = text_recognize()
	rospy.on_shutdown(text_recognize.onShutdown)
	rospy.spin()
