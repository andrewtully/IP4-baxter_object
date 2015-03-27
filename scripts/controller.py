#!/usr/bin/python

import rospy
import sys

from image_receiver import ImageReceiver
from move_controller import MoveController
from image_processor import ImageProcessor
from training import Training
from query import Query

class Controller(object):
	def __init__(self, setup=False):
		rospy.init_node("Baxter_Object")
		
		
		self.right_camera = ImageReceiver("right_hand_camera")
		self.left_camera = ImageReceiver("left_hand_camera")
		"""
		self.right_camera.enable_camera()
		self.left_camera.enable_camera()
		"""
		distortion, camera_matrix = self.left_camera.get_intrinsics()
		
		self.move_left = MoveController("left")
		self.move_right = MoveController("right")
		self.image_proc = ImageProcessor(self.move_left.home, camera_matrix, distortion)
		
		self.objects = []
		
		if (setup):
			self.move_right.move_to_pose((0.7808787738681676, -0.6091393167320737, 0.2526229980287372))
			self.move_left.update_table_height()
			setup_file = open("setup.txt", "w")
			setup_file.write("Table_Height:" + str(self.move_left.table_height) + "\n")
			
			setup_file.close()
		else:
			try:
				setup_file = open("setup.txt", "r")
				line = setup_file.readline()
				
				while (line != ""):
					split_line = line.split(":")
					if (split_line[0] == "Table_Height"):
						self.move_left.table_height = float(split_line[-1])
						self.image_proc.table_height = float(split_line[-1])
						break
					
					line = setup_file.readline()
						
				setup_file.close()
			except:
				print "No existing 'setup.txt' file: Please create file or run without '-setup' flag)"
		
		self.update_home_pose(self.move_left.home)

################################################################################
	
	def update_home_pose(self, pose):
		self.image_proc.update_home_pose(pose)
		self.move_left.home = pose

	def survey_scene(self):
		pos = self.move_left.home
		y = pos[1]
		
		#for new_y in range(y - 0.2, y + 0.2, 0.1):
		#	self.move_left.move_to_pose((pos[0], new_y, pos[2]))
		new_y = y
		while (new_y <= (y + 0.4)):
			print new_y
			self.move_left.move_to_pose((pos[0], new_y, pos[2]))
			self.get_current_pos_image(self.left_camera)
			new_y += 0.2	
		
		new_y = y - 0.4
		while (new_y <= y):
			print new_y
			self.move_right.move_to_pose((pos[0], new_y, pos[2]))
			self.get_current_pos_image(self.right_camera)
			new_y += 0.2	
		
		

################################################################################

	def get_home_image(self):
		self.move_left.move_to_home()
		rospy.sleep(0.1)

		img = self.left_camera.get_image()

		self.image_proc.set_image(img)
	
	def get_current_pos_image(self, camera):
		rospy.sleep(0.1)
		img = camera.get_image()
		self.image_proc.set_image(img)
		
################################################################################
	
	def current_position_objects(self):
		image_proc.contours_from_image()
		
		for contour in image_proc.img_contours:
			self.objects.append(DetectedObject(contour, imag_proc.cv_img))	
		

if __name__ == "__main__":
	if (len(sys.argv) > 1 and sys.argv[1] == "-setup"):
		controller = Controller(True)
	elif (len(sys.argv) == 3 and sys.argv[1] == "-train"):
		controller = Controller()
		controller.move_right.move_to_pose((0.7808787738681676, -0.6091393167320737, 0.2526229980287372))	
		training = Training(sys.argv[2], controller.move_left, controller.image_proc, controller.left_camera)
	else:
		controller = Controller()
		query = Query(controller.move_left, controller.left_camera, controller.image_proc)
		
	#controller.home_frame_average()
	#controller.current_position_objects
	#controller.survey_scene()
	
