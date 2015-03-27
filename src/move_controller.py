#!/usr/bin/python

import rospy
import baxter_interface
from math import pi

from geometry_msgs.msg import (
	Pose,
	Point,
	Quaternion,
	PoseStamped,
)

from std_msgs.msg import Header
from sensor_msgs.msg import Range

from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

from baxter_ikhelper import IKHelper

class MoveController(object):
	""" Controls the movement of Baxter's limbs """

	home = (0.7808787738681676-0.2, 0.0091393167320737, 0.2526229980287372 - 0.2)

################################################################################

	def __init__(self, arm):
		self._ik_helper = IKHelper(arm)
	
		self.ns = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
		self.ik_serv = rospy.ServiceProxy(self.ns, SolvePositionIK)
		self.ik_req = SolvePositionIKRequest()
		self.header = Header(stamp=rospy.Time.now(), frame_id="base")
		
		self.arm = baxter_interface.Limb(arm)
		self.arm.set_joint_position_speed(0.95)
		self.gripper = baxter_interface.Gripper(arm)
		self.gripper.calibrate()
		
		self.infrared_topic = "/robot/range/" + arm + "_hand_range/state"
		self.infrared_sub = None
		
		self.table_height = None
	
################################################################################

	""" Move Baxter back to his home position """
	def move_to_home(self):
		return self.move_to_pose(self.home)
	
	""" Move Baxter to specified position """
	def move_to_pose(self, pos):
		return self._ik_helper.set_arm(pos)
			
################################################################################

	""" Return the current position of the arm in use """
	def current_position(self):
		return self._ik_helper.get_arm()

################################################################################

	def _ir_callback(self, data):
		if ((data.range > data.min_range) and (data.range < data.max_range)):
			self.table_height = data.range
		else:
			self.table_height = False
		
		self.infrared_sub.unregister()

	""" Get the table height by moving the arm down in stages """
	def _infrared_table_height(self):
		self.table_height = None
		self.infrared_sub = rospy.Subscriber(self.infrared_topic, Range, self._ir_callback)
		
		while (self.table_height == None):
			continue
		
		return self.table_height

	""" Reclculates the table height """
	def update_table_height(self):
		self.move_to_home()
		
		while (self._infrared_table_height() is False):
			pos = self._ik_helper.get_arm()
			self.move_to_pose((pos.x, pos.y, (pos.z - 0.2)))
		

		self.table_height = self.home[2] - self.table_height - 0.01
		self.move_to_home()
