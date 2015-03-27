#!/usr/bin/python

import rospy
import cv2

import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

DEFAULT_PIXELS_PER_METRE = 15 / 2.54*100
DEFAULT_HOME_HEIGHT = 25.875 * 2.54 / 100

class ImageProcessor(object):
	""" Class that performs any image vision techniques """

	def __init__(self, home_pose, camera_matrix, distortion, table_height=None, cv_img=None):
		self.home_pose = home_pose
		self.camera_matrix = camera_matrix
		self.distortion = distortion
		
		if (table_height is None):
			self.table_height = home_pose[2] - DEFAULT_HOME_HEIGHT
		else:
			self.table_height = table_height
		
		self.bridge = CvBridge()
		self.image_topic = "/robot/xdisplay"
		
		self.set_image(cv_img)
		
		self.segment_img = None
		self.img_contours = None
		self.img_hierarchy = None
		
		self.pixels_per_metre = DEFAULT_HOME_HEIGHT / (home_pose[2] - self.table_height) * DEFAULT_PIXELS_PER_METRE
	
################################################################################

	""" Publish the image to Baxter's screen """
	def display_image(self, img, encoding):
		img = cv2.resize(img, (1024, 600), interpolation=cv2.INTER_AREA)
		msg = self.bridge.cv_to_imgmsg(cv2.cv.fromarray(img), encoding)
		
		self.img_pub = rospy.Publisher("/robot/xdisplay", Image, latch=True)
		self.img_pub.publish(msg)
		rospy.sleep(1)

	""" Write the image to a specified location """
	def write_image(self, location):
		cv2.imwrite(location, self.cv_img)
	
	""" Write the Canny image to a specified location """
	def write_segmented_image(self, location):
		cv2.imwrite(location, self.segment_img)

################################################################################

	def _undistort_image(self, cv_img):
		height, width, _ = cv_img.shape
		cam_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.distortion, (width, height), 1, (width, height))
		undistort = cv2.undistort(cv_img, self.camera_matrix, self.distortion, None, cam_matrix)
		
		x, y, w, h = roi
		return np.array(undistort[y:y+h, x:x+w])


	""" Set the OpenCV (BGR) image to be used in this class """
	def set_image(self, cv_img, undistort=True):
		if (cv_img is None):
			self.cv_img = None
			self.img_width = 0
			self.img_height = 0
		else:	
			if (undistort):
				self.cv_img = self._undistort_image(cv_img)
			else:
				self.cv_img = cv_img	
		
			self.cv_width, self.cv_height = cv2.cv.GetSize(cv2.cv.fromarray(self.cv_img))


################################################################################
	###############################
	### OpenCV helper functions ###
	###############################

	def _bgr_to_grey(self, img):
		return cv2.cvtColor(img, cv2.cv.CV_BGR2GRAY)

	def _grey_to_bgr(self, img):
		return cv2.cvtColor(img, cv2.cv.CV_GRAY2BGR)

	def _greyscale_blur(self):
		blur = None
		
		if (self.cv_img is not None):
			grey = self._bgr_to_grey(self.cv_img)
			blur = cv2.blur(grey, (3,3))
	
		return blur

	def _canny(self, grey):
		# Use Otsu's thresholding to dynamically calculate threshold)
		thresh, _ = cv2.threshold(grey, 0, 255, cv2.cv.CV_THRESH_BINARY | cv2.cv.CV_THRESH_OTSU)
		return cv2.Canny(grey, thresh, thresh*2)

################################################################################
	
	""" Get the image from the specified camera """
	def get_image(self, camera):
		img = camera.get_image()
		self.set_image(img)
	
	""" Perform the edge detection """
	def edges(self):
		self.segment_img = self._canny(self._bgr_to_grey(self.cv_img))
		
	""" Perform the edge detection using drame averaging to reduce noise """	
	def frame_average_edges(self, frames, camera):
		self.get_image(camera)
		
		avg = np.zeros(self.cv_img.shape, np.float32)
		weight = float(1) / frames
		
		for i in range(frames):
			blur = self._greyscale_blur()
				
			edges = self._canny(blur)
			edges = self._grey_to_bgr(edges)
			
			cv2.accumulateWeighted(edges, avg, 0.1)
			
			img = camera.get_image()
			self.set_image(img)
		
		segment_img = cv2.convertScaleAbs(avg)
		segment_img = self._bgr_to_grey(segment_img)
		self.segment_img = cv2.morphologyEx(segment_img, cv2.MORPH_CLOSE, (40,40))
		
################################################################################

	""" Returns a masked image based on the input contours """
	def mask_contour(self, contour_id):
		mask = np.zeros((self.cv_img.shape[0], self.cv_img.shape[1], 1), np.uint8)
		cv2.drawContours(mask, self.img_contours, contour_id, 255, cv2.cv.CV_FILLED)
		return cv2.bitwise_and(self.cv_img, self.cv_img, mask=mask)

################################################################################
	
	""" Returns the area of bounding rectangle """
	def _rect_area(self, rect):
		return rect[2] * rect[3]
 
 	""" Returns a mask of the bounding rectangle with the smallest area (i.e. the object) """
	def _bounding_rect(self, img, contours):
		bound_rect = []
		for i in range(len(contours)):
			bound_rect.append(cv2.boundingRect(contours[i]))
     
		bound = None
		if (len(bound_rect) > 1):
			min_area = None
			for rect in bound_rect:
				area = self._rect_area(rect)
         
				if (min_area is None or area < min_area):
					min_area = area
     
			for rect in bound_rect:
				if (self._rect_area(rect) == min_area):
					bound = rect
		elif (len(bound_rect) == 1):
			bound = bound_rect[0]
 		else:
 			return
 		
		tl = (bound[0], bound[1])  
		br = (bound[0] + bound[2], bound[1] + bound[3])
 
		mask = np.zeros((img.shape[0], img.shape[1], 1), np.uint8)
		cv2.rectangle(mask, tl, br, 255, cv2.cv.CV_FILLED, 8, 0)
		return mask
 
 	""" Sets the OpenCV image to be a bounding rectangle mask of object in scene """
	def bounding_rectangle_mask(self):
		grey = cv2.cvtColor(self.cv_img, cv2.cv.CV_BGR2GRAY)
 
		blur = cv2.GaussianBlur(grey, (15, 15), 0)
		thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 1)
 
		kernel = np.ones((3, 3), np.uint8)
		closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=4)
 
		poss_contours, hierarchy = cv2.findContours(closing, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE, (0, 0))
 
		contours = []
		for cnt in poss_contours:
			area = cv2.contourArea(cnt)
     
			if area < 4000:
				continue
 
			if len(cnt) < 5:
				continue
 
			contours.append(cnt)
  
		mask = self._bounding_rect(self.cv_img, contours)
		self.cv_img = cv2.bitwise_and(self.cv_img, self.cv_img, mask=mask)
	
################################################################################

	""" Calculates the contours from the segmented image """
	def contours_from_image(self):
		if (self.segment_img is not None):
			poss_contours, self.img_hierarchy = cv2.findContours(self.segment_img, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_NONE, (0, 0))
	
			self.img_contours = []
			for i in range(0, len(poss_contours)):
				area = cv2.contourArea(poss_contours[i])
				if (area > 1500):
					self.img_contours.append(poss_contours[i])

	
	""" Calculates the mass centre of a contour """
	def _calculate_mass_centre(self, contour):
		moments = cv2.moments(contour, False)	
		return (float(moments["m10"])/float(moments["m00"])), (float(moments["m01"])/float(moments["m00"]))
				
	""" Helper function to draw the contours on an image """		
	def draw_contours(self):
		if (self.img_contours is not None):
			img = np.copy(self.cv_img)
			
			for i in range(len(self.img_contours)):
				cv2.drawContours(img, self.img_contours, i, (0,255,0), 3, 8, self.img_hierarchy, 0, (0,0))
			
			return img
	
	""" Draws the specified text on the contour at the mass centre """
	def draw_object_text(self, img, obj_dict):
		if (self.img_contours is not None):
			keys = obj_dict.keys()
			for i in range(len(self.img_contours)):
				if (i in keys):
					text = obj_dict[i]
					thickness = 2
					
					ret, baseline = cv2.getTextSize(text, cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, 0)
					
					point_x, point_y = self._calculate_mass_centre(self.img_contours[i])
					point_x = int(point_x - (ret[0]/2))
					point_y = int(point_y)
					
					cv2.putText(img, text, (point_x, point_y), cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), thickness, 8)
		
		return img
			
		
################################################################################

	def update_home_pose(self, pose):
		self.home_pose = pose
		self.pixels_per_metre = DEFAULT_HOME_HEIGHT / (self.home_pose[2] - self.table_height) * DEFAULT_PIXELS_PER_METRE
	

