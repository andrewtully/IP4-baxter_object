#!/usr/bin/python

import rospy

import numpy as np

from baxter_interface.camera import CameraController
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo

class ImageReceiver(object):
	""" Gets the image from the camera and converts it into a OpenCV compatible (BGR) image """

	def __init__(self, camera_name):
		self.camera_name = camera_name
		self.camera_controller = CameraController(camera_name)
		self.camera_controller.resolution = (960, 600)
		self.camera_controller.exposure = 99
		
		self.bridge = CvBridge()
		
		self.cv_img = None
		self.ros_img = None
		
		self.img_topic = "/cameras/" + camera_name + "/image"
		self.img_sub = None
		
		self.intrinsics_topic = "/cameras/" + camera_name + "/camera_info"
		self.intrinsics_sub = None
		
	
################################################################################
	
	""" Enables the camera """
	def enable_camera(self):
		self.camera_controller.open()
	
################################################################################	

	""" Disables the camera """
	def disable_camera(self):
		self.camera_controller.close()

################################################################################
	
	def _intrinsics_callback(self, data):
		mtx = []
		mtx.append([data.K[0], data.K[1], data.K[2]])
		mtx.append([data.K[3], data.K[4], data.K[5]])
		mtx.append([data.K[6], data.K[7], data.K[8]])
		
		self.distortion = np.array(data.D)
		self.matrix = np.array(mtx)		

	""" Returns the distortion and camera matrix of the camera """
	def get_intrinsics(self):
		self.distortion = None
		self.matrix = None
		self.intrinsics_sub = rospy.Subscriber(self.intrinsics_topic, CameraInfo, self._intrinsics_callback)
		
		while (self.distortion is None):
			continue
			
		self.intrinsics_sub.unregister()
		return self.distortion, self.matrix
		

################################################################################

	def _image_callback(self, data):
		self.ros_img = data
		try:
			self.cv_img = np.asarray(self.bridge.imgmsg_to_cv(data, "bgr8"))
		except CvBridgeError, e:
			print e
	
	""" Returns the image from the subscribed camera topic """
	def get_image(self):
		self.cv_img = None
		self.ros_img = None

		self.img_sub = rospy.Subscriber(self.img_topic, Image, self._image_callback)

		while (self.cv_img is None):
			continue

		img = self.cv_img
		self.img_sub.unregister()
		
		return img
