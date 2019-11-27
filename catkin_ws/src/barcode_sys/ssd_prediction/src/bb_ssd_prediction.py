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
from nav_msgs.msg import Path
from cv_bridge import CvBridge, CvBridgeError
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable
from ssd import build_ssd
from u_network import UNet
import os 
import message_filters

class bb_ssd_prediction(object):
	def __init__(self):
		self.prob_threshold = 0.95
		self.cv_bridge = CvBridge() 
		r = rospkg.RosPack()
		self.path = r.get_path('ssd_prediction')

		self.labels = ['background', 'barcode']
		self.objects = []
		self.network = build_ssd('test', 300, len(self.labels)) 
		self.is_compressed = False
		self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
		self.cuda_use = torch.cuda.is_available()
		self.u_net = UNet(n_channels=3, n_classes=1)
		self.u_net.to(device=self.device)
		model_name = "UNet.pkl"
		state_dict = torch.load(os.path.join(self.path, "weights/", model_name))
		self.u_net.load_state_dict(state_dict)

		if self.cuda_use:
			self.network = self.network.cuda()
		model_name = "barcode.pth"
		state_dict = torch.load(os.path.join(self.path, "weights/", model_name))
		self.network.load_state_dict(state_dict)
		#### Publisher
		self.image_pub = rospy.Publisher("~predict_img", Image, queue_size = 1)
		self.mask_pub = rospy.Publisher("~predict_mask", Image, queue_size = 1)

		### msg filter 

		video_mode = False 
		if video_mode:
			image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.video_callback)
		else:
			image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
			depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
			ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
			ts.registerCallback(self.callback)

	def callback(self, img_msg, depth):
		try:
			if self.is_compressed:
				np_arr = np.fromstring(img_msg.data, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		img = cv_image.copy()
		
		(rows, cols, channels) = cv_image.shape	
		rows = int(np.ceil(rows/32.)*32)
		cols = int(np.ceil(cols/32.)*32)
		cv_image1 = np.zeros((rows, cols, channels),dtype = np.uint8)
		cv_image1[:cv_image.shape[0],:cv_image.shape[1],:cv_image.shape[2]] = cv_image[:,:,:]
		cv_image = cv_image1.copy()
		
		self.width = cols
		self.height = rows
		predict_img, obj_list, mask = self.predict(cv_image)
		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
			self.mask_pub.publish(self.cv_bridge.cv2_to_imgmsg(mask, "8UC1"))
		except CvBridgeError as e:
			print(e)

		for obj in obj_list:
			obj[0] = obj[0] - 5
			obj[1] = obj[1] - 5
			obj[2] = obj[2] + 10
			obj[3] = obj[3] + 10

			mask = np.zeros((rows, cols), dtype = np.uint8)
			point_list = [(int(obj[0]), int(obj[1])),(int(obj[0] + obj[2]),int(obj[1])),\
				(int(obj[0] + obj[2]), int(obj[1] + obj[3])), (int(obj[0]), int(obj[1] + obj[3]))]

			cv2.fillConvexPoly(mask, np.asarray(point_list,dtype = np.int), 255)

	def video_callback(self, img_msg):

		try:
			if self.is_compressed:
				np_arr = np.fromstring(img_msg.data, np.uint8)
				cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			else:
				cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
			print(e)
		img = cv_image.copy()
		

		(rows, cols, channels) = cv_image.shape
		self.width = cols
		self.height = rows
		predict_img, obj_list = self.predict(cv_image)
		try:
			self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(predict_img, "bgr8"))
		except CvBridgeError as e:
			print(e)

		for obj in obj_list:
			mask = np.zeros((rows, cols), dtype = np.uint8)
			point_list = [(int(obj[0]), int(obj[1])),(int(obj[0] + obj[2]),int(obj[1])),\
				(int(obj[0] + obj[2]), int(obj[1] + obj[3])), (int(obj[0]), int(obj[1] + obj[3]))]

			cv2.fillConvexPoly(mask, np.asarray(point_list,dtype = np.int), 255)
			mask = self.cv_bridge.cv2_to_imgmsg(mask, "8UC1")
			self.mask_pub.publish(mask)

	def predict(self, img):
		mask = np.zeros(img.shape[:2], dtype = np.uint8)
		# Preprocessing
		image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
		x = cv2.resize(image, (300, 300)).astype(np.float32)
		x -= (104.0, 117.0, 123.0)
		x = x.astype(np.float32)
		x = x[:, :, ::-1].copy()
		x = torch.from_numpy(x).permute(2, 0, 1)

		#SSD Forward Pass
		xx = Variable(x.unsqueeze(0))     # wrap tensor in Variable
		if self.cuda_use:
			xx = xx.cuda()
		time = rospy.get_time()
		y = self.network(xx)
		print(1./(rospy.get_time()- time))
		scale = torch.Tensor(img.shape[1::-1]).repeat(2)
		detections = y.data	# torch.Size([1, 4, 200, 5]) --> [batch?, class, object, coordinates]
		objs = []
		for i in range(detections.size(1)): # detections.size(1) --> class size
			for j in range(5):	# each class choose top 5 predictions
				if detections[0, i, j, 0].numpy() > self.prob_threshold:
					score = detections[0, i, j, 0]
					pt = (detections[0, i, j,1:]*scale).cpu().numpy()
					objs.append([pt[0], pt[1], pt[2]-pt[0]+1, pt[3]-pt[1]+1, i])
					
					### U net region
					min_x = int(pt[0]) if int(pt[0]) > 0 else 0
					min_y = int(pt[1]) if int(pt[1]) > 0 else 0
					max_x = int(pt[2]) if int(pt[2]) < 640 else 640
					max_y = int(pt[3]) if int(pt[3]) < 480 else 480

					img_box = image[min_y:max_y, min_x:max_x].copy()
					h_b, w_b, _ = img_box.shape
					img_box = cv2.resize(img_box, (64, 64)).astype(np.float32)
					img_box -= (103.939, 116.779, 123.68)
					img_box = img_box[:, :, ::-1].copy()
					img_box = torch.from_numpy(img_box).permute(2, 0, 1)
					img_box = Variable(img_box.unsqueeze(0)) 
					inputs = img_box.to(device=self.device, dtype=torch.float32)
					
					output = self.u_net(inputs)
					output = output.data.cpu().numpy()
					N, _, h, w = output.shape
					pred = output.transpose(0, 2, 3, 1).reshape(-1, 1).reshape(N, h, w)[0]
					pred[pred < 0] = 0 
					pred[pred > 0] = 255
					pred = cv2.resize(pred, (w_b,h_b))
					mask[min_y:max_y, min_x:max_x] = pred


		for obj in objs:
			if obj[4] == 1:
				color = (0, 255, 255)
			elif obj[4] == 2:
				color = (255, 255, 0)
			elif obj[4] == 3:
				color = (255, 0, 255)
			else:
				color = (0, 0, 0)
			cv2.rectangle(img, (int(obj[0]), int(obj[1])),\
								(int(obj[0] + obj[2]), int(obj[1] + obj[3])), color, 3)
			cv2.putText(img, self.labels[obj[4]], (int(obj[0] + obj[2]), int(obj[1])), 0, 1, color,2)


		return img, objs, mask


	def onShutdown(self):
		rospy.loginfo("Shutdown.")	


	def getXYZ(self,xp, yp, zc):

		inv_fx = 1.0/self.fx
		inv_fy = 1.0/self.fy
		x = (xp-self.cx) *  zc * inv_fx
		y = (yp-self.cy) *  zc * inv_fy
		z = zc
		return (x,y,z)			


if __name__ == '__main__': 
	rospy.init_node('bb_ssd_prediction',anonymous=False)
	bb_ssd_prediction = bb_ssd_prediction()
	rospy.on_shutdown(bb_ssd_prediction.onShutdown)
	rospy.spin()
