#!/usr/bin/python
import rospy
from image_geometry import PinholeCameraModel, StereoCameraModel
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

import tf
import cv2
import pickle
import os

from detected_object import DetectedObject
""" Trains an object in the scene """

class Training(object):
	OUTPUT_LOCATION= "./train/"

	def __init__(self, name, move, img_proc, camera):
		self.name = name
		
		self.move = move
		self.img_proc = img_proc
		self.camera = camera
		
		self.views = []
		
		self.centre = self._mass_centre()
		
		self._all_viewpoints()
		self._create_output_folder()
		self._store_keypoints()
		
	############################################################################
	
	""" If OUTPUT_LOCATION deosn't exist, create the folder """
	def _create_output_folder(self):
		if (not os.path.isdir(self.OUTPUT_LOCATION)):
			os.makedirs(self.OUTPUT_LOCATION)
	
	############################################################################

	""" Find mass centre of object and hover over """
	def _mass_centre(self):
		self.move.move_to_home()
		rospy.sleep(0.1)
		
		self.img_proc.frame_average_edges(100, self.camera) 
		
		self.img_proc.contours_from_image()
		
		if (len(self.img_proc.img_contours) != 1):
			print str(len(self.img_proc.img_contours)) + " objects detected on the table"
			return
		
		obj = DetectedObject(self.img_proc.img_contours[0], self.img_proc.cv_img)
		
		self.pcm = PinholeCameraModel()
		self._get_intrinsics()

		pos = self.pcm.projectPixelTo3dRay(obj.mass_centre)	
		new_pos = (pos[0], pos[1], self.move.current_position()[2])
		curr = self.move.current_position()
		
		
		#self.move.move_to_pose(new_pos)
		""" Baxter needs calibrated !! """
		return self.move.current_position()
		
	def _get_intrinsics(self):
		self.intrinsics_sub = rospy.Subscriber(self.camera.intrinsics_topic, CameraInfo, self.pcm.fromCameraInfo)
		
		rospy.sleep(1)
		
		self.intrinsics_sub.unregister()

	 
################################################################################
	
	""" Create a mask based on the bounding box of object """
	def _object_mask(self, i):
		self.img_proc.get_image(self.camera)
		cv2.imwrite(self.OUTPUT_LOCATION + self.name + "/" + self.name + str(i) + ".jpg", self.img_proc.cv_img)
		self.img_proc.bounding_rectangle_mask()

	def _collect_viewpoint_quadrant(self, x, y, pos, deltaX, deltaY, i):
		pos = (pos[0]+x, pos[1]+y, pos[2]) 
		self.move.move_to_pose(pos)
		rospy.sleep(1)
		
		self._object_mask(i)
		return (x + deltaX), (y + deltaY)
	
	""" Move the arm around the object collecting different images """
	def _all_viewpoints(self):
		pos = self.centre
		x = 0.0
		y = 0.1
		counter = 1
        
		self._object_mask(0)
		#self.img_proc.write_image("train/" + self.name + "/" + self.name + "0.jpg")
		self.views.append(self.img_proc.cv_img)
        
		while (y > 0):
			x, y = self._collect_viewpoint_quadrant(x, y, self.centre, -0.02, -0.02, counter)
			#self.img_proc.write_image("train/" + self.name + "/" + self.name + str(counter) + ".jpg")
			self.views.append(self.img_proc.cv_img)
			self.move.move_to_pose(self.centre)
			counter += 1
		
		while (x < 0):
			x, y = self._collect_viewpoint_quadrant(x, y, self.centre, 0.02, -0.02, counter)
			#self.img_proc.write_image("train/" + self.name + "/" + self.name + str(counter) + ".jpg")
			self.views.append(self.img_proc.cv_img)
			self.move.move_to_pose(self.centre)
			counter += 1

		while (y < 0):
			x, y = self._collect_viewpoint_quadrant(x, y, self.centre, 0.02, 0.02, counter)
			#self.img_proc.write_image("train/" + self.name + "/" + self.name + str(counter) + ".jpg")
			self.views.append(self.img_proc.cv_img)
			self.move.move_to_pose(self.centre)
			counter += 1

		while (x > 0):
			x, y = self._collect_viewpoint_quadrant(x, y, self.centre, -0.02, 0.02, counter)
			#self.img_proc.write_image("train/" + self.name + "/" + self.name + str(counter) + ".jpg")
			self.views.append(self.img_proc.cv_img)
			self.move.move_to_pose(self.centre)
			counter += 1

################################################################################
	
	""" Extract SURF keypoints and descriptors """
	def surf(self, img):
		grey = cv2.cvtColor(img, cv2.cv.CV_BGR2GRAY)
     
		minHessian = 400
		surf = cv2.SURF(minHessian)
     
		return surf.detectAndCompute(grey,None)
		
	""" Put the keypoints into a format which can be printed to a file """
	def _serialize_keypoints(self, kp_array, kp, des):
		curr_kp = []

		for i in range(len(kp)):
			point = kp[i]
			curr_kp.append((point.pt[0], point.pt[1], point.size, point.angle, point.response, point.octave, point.class_id, des[i])) 
		kp_array.append(curr_kp)
		return kp_array
	
	""" Store the keypoints and descriptors in a file in OUTPUT_LOCATION """
	def _store_keypoints(self):
		kp_array = []
		for img in self.views:
			kp, des = self.surf(img)
			kp_array = self._serialize_keypoints(kp_array, kp, des)
		
		f = file(self.OUTPUT_LOCATION + self.name + ".dat", "w")
		pickle.dump(kp_array, f)
		f.close()
