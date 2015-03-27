#!/usr/bin/python

import rospy
import sys

from image_receiver import ImageReceiver
from move_controller import MoveController
from image_processor import ImageProcessor
from training import Training
from query import Query

class Controller(object):
	""" Main script that parses the inputs and calls teh relevant classes """
	def __init__(self, setup=False):
		rospy.init_node("Baxter_Object")
		
		self.right_camera = ImageReceiver("right_hand_camera")
		self.left_camera = ImageReceiver("left_hand_camera")
		
		self.right_camera.enable_camera()
		self.left_camera.enable_camera()
		
		distortion, camera_matrix = self.left_camera.get_intrinsics()
		
		self.move_left = MoveController("left")
		self.move_right = MoveController("right")
		self.image_proc = ImageProcessor(self.move_left.home, camera_matrix, distortion)
		
		""" Setup find height of table """
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
				print "No existing 'setup.txt' file: Please create file or run with '-setup' flag)"
		
		self.update_home_pose(self.move_left.home)

################################################################################
	
	def update_home_pose(self, pose):
		self.image_proc.update_home_pose(pose)
		self.move_left.home = pose
		
################################################################################
		
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
		
	
