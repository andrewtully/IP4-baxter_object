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
	"""home_pose = Pose(
					position = Point(x=0.52598, y=-0.3365, z=0.45,),
					orientation = Quaternion(x=0, y=pi/4, z=0, w=0),
				)"""
	#home = (0.52598, 0.3365, 0.45)
	#home = (0.7808787738681676, 0.2091393167320737, 0.2526229980287372)
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
		#self.gripper.calibrate()
		
		self.infrared_topic = "/robot/range/" + arm + "_hand_range/state"
		self.infrared_sub = None
		
		self.table_height = None
	
################################################################################
	
	"""
	def move_to_pose(self, pose_list, move=True):
		self.ik_req.pose_stamp = []
		for pose in pose_list:
			pose_stamped = PoseStamped(header=self.header, pose=pose)
			self.ik_req.pose_stamp.append(pose_stamped)
		try:
			rospy.wait_for_service(self.ns, 5.0)
			resp = self.ik_serv(self.ik_req)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return -1

		for i in xrange(len(resp.isValid)):
			if (resp.isValid[i]):
				print "Success:- Valid joint solution is found"
				
				## Is move needed here? ##
				if move:
					limb_joints = dict(zip(resp.joints[i].name, resp.joints[i].position))
					self.arm.move_to_joint_positions(limb_joints)
					result = 0
			else:
				print "No valid joint solution was found"
				result = -1		

			return result
	"""
	
################################################################################

	def move_to_home(self):
		#return self.move_to_pose([self.home_pose])
		return self.move_to_pose(self.home)
	
	def move_to_pose(self, pos):
		return self._ik_helper.set_arm(pos)
			
################################################################################

	def current_position(self):
		return self._ik_helper.get_arm()

################################################################################

	def _ir_callback(self, data):
		if ((data.range > data.min_range) and (data.range < data.max_range)):
			self.table_height = data.range
		else:
			self.table_height = False
		
		self.infrared_sub.unregister()

	def infrared_table_height(self):
		self.table_height = None
		self.infrared_sub = rospy.Subscriber(self.infrared_topic, Range, self._ir_callback)
		
		while (self.table_height == None):
			continue
		
		return self.table_height

################################################################################

	"""def new_home_pose(self):
		return Pose(
					position = Point(x=0.52598, y=-0.3365, z=0.45,),
					orientation = Quaternion(x=0, y=pi/4, z=0, w=0),
				)
	"""

	## If it doesn't work, this could be where it fails
	def update_table_height(self):
		#pose = self.home_pose
		self.move_to_home()
		"""temp = self.new_home_pose()
		self.move_to_pose([temp])"""
		
		while (self.infrared_table_height() is False):
			#temp.position.z = temp.position.z - 0.2
			pos = self._ik_helper.get_arm()
			self.move_to_pose((pos.x, pos.y, (pos.z - 0.2)))
		

		#self.table_height = temp.position.z - self.table_height - 0.01
		self.table_height = self.home[2] - self.table_height - 0.01
		self.move_to_home()
