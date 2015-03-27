import rospy

import os
import pickle
import cv2
import numpy as np

class Query(object):
	DATA_LOCATION= "./train/"

	def __init__(self, move, camera, img_proc):
		self.move = move
		self.camera = camera
		self.img_proc = img_proc
		
		self.obj_dict = {}
		self.output_img = None	
		
		self.home_frame_average()
		#self.display_image(self.img_proc.cv_img)
		self.test()
		
		self.show_contours()
		self.display_image(self.output_img)
		cv2.imwrite("output.jpg", self.output_img)
	
	############################################################################
	
	def home_frame_average(self):
		self.move.move_to_home()
		rospy.sleep(0.1)
		
		self.img_proc.frame_average_edges(100, self.camera)
		self.img_proc.get_image(self.camera) 
		#self.img_proc.write_image("image7.jpg")
		#self.img_proc.write_segmented_image("segment7.jpg")
	
	############################################################################
	
	def _load_keypoints(self, path):
		f = file(path, "r")
		kp_file = pickle.load(f)
		
		kp_dict = {"kp":[], "des":[]}
		for pt in kp_file:
			
			kp_dict["kp"].append(cv2.KeyPoint(x=pt[0],
                                      y=pt[1],
                                      _size=pt[2],
                                      _angle=pt[3],
                                      _response=pt[4],
                                      _octave=pt[5],
                                      _class_id=pt[6]))
			kp_dict["des"].append(pt[7])
	
		return kp_dict
	
	############################################################################
	
	def surf(self, img):
		grey = cv2.cvtColor(img, cv2.cv.CV_BGR2GRAY)
     
		minHessian = 400
		surf = cv2.SURF(minHessian)
     
		return surf.detectAndCompute(grey,None)
	
	def perform_matching(self, kp_scene, des_scene, kp_dict):
		max_match = 0
		best_match = None
		
		keys = kp_dict.keys()
		
		for key in keys:
			kp_object = kp_dict[key]["kp"]
			des_object = kp_dict[key]["des"]
			
			FLANN_INDEX_KDTREE = 0
			index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
			search_params = dict(checks=50)
			
			flann = cv2.FlannBasedMatcher(index_params, search_params)
			matches = flann.knnMatch(np.array(des_object),des_scene,k=2)
			
			good = []
			for m,n in matches:
				if m.distance < 0.7*n.distance:
					good.append(m)
			
			# Match not considered strong enough
			if (len(good) < 25):
			 	continue
			 
			if (len(good) > max_match):
				max_match = len(good)
				best_match = key
		
		return best_match
			
	
	def match(self, contour_id, kp_dict):
		scene = self.img_proc.mask_contour(contour_id)
		kp, des = self.surf(scene)
		
		match_object = self.perform_matching(kp, des, kp_dict)
		
		if match_object is not None:
			self.obj_dict[contour_id] = match_object
	
	def test(self):
		self.img_proc.contours_from_image()
		
		kp_files = []
		kp_dict = {}
		
		for dirpath, dnames, fnames in os.walk(self.DATA_LOCATION):
			for f in fnames:
				if (f.endswith(".dat")):
					kp_files.append(f)
					kp_dict[f.split(".")[0]] = self._load_keypoints(self.DATA_LOCATION + f)
			
		for i in range(len(self.img_proc.img_contours)):
			self.match(i, kp_dict) 
	
	############################################################################
	
	def display_image(self, img):
		self.img_proc.display_image(self.output_img, "bgr8")
	
	def show_contours(self):
		if (self.img_proc.img_contours is not None):
			if (len(self.img_proc.img_contours) > 0):

				self.output_img = self.img_proc.draw_contours()
				self.output_img = self.img_proc.draw_object_text(self.output_img, self.obj_dict)
			else:
				print "No objects detected on table"
				self.output_img = self.img_proc.cv_img
		
		
		
