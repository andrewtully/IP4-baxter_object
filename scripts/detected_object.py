#!/usr/bin/python

import cv2
import numpy as np

class DetectedObject(object):
	""" For an object detected, claculate various features """

	def __init__(self, contour, original_img):
		self.contour = contour
		self.moments = cv2.moments(self.contour, False)	
		
		self.area = self.moments["m00"]
		self.length = cv2.arcLength(contour, True)
		self.mass_centre = self._calculate_mass_centre()		
		
		self.b_colour_hist, g_colour_hist, r_colour_hist = self._calculate_colour_histograms(original_img)

################################################################################

	def _calculate_mass_centre(self):
		return (float(self.moments["m10"])/float(self.moments["m00"])), (float(self.moments["m01"])/float(self.moments["m00"]))

	
	def _histogram_mask(self, array, mask):
		hist_size = 256
		poss_range = [0, 256]
		rows = mask.shape[0]
	
		hist = cv2.calcHist([array], [0], mask, [hist_size], poss_range, False)
		return cv2.normalize(hist, 0, rows, cv2.NORM_MINMAX)
		
	def _calculate_colour_histograms(self, img):
		r, h, _ = img.shape
		mask = np.zeros((r, h, 1), np.uint8)
		cv2.drawContours(mask, self.contour, -1, 255, cv2.cv.CV_FILLED)
		
		bgr_split = cv2.split(img)
		
		return self._histogram_mask(bgr_split[0], mask), self._histogram_mask(bgr_split[1], mask), self._histogram_mask(bgr_split[2], mask)
	
################################################################################

