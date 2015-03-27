#!/usr/bin/python

import math
import rospy

from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from baxter_interface import RobotEnable, CameraController, Limb
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class IKHelper(object):
	"""An abstraction layer for using Baxter's built in IK service."""

	############################################################################

	def __init__(self, arm):		
		self._arm = Limb(arm)
		self._arm.set_joint_position_speed(0.3)

		self._arm_iksvc = rospy.ServiceProxy(
			'ExternalTools/' + arm + '/PositionKinematicsNode/IKService',
			SolvePositionIK
		)

		self.joint_update_pub = rospy.Publisher(
			'/robot/joint_state_publish_rate', 
			UInt16
		)

	############################################################################

	""" Move arm to neutral position """   
	def reset(self):
		self._arm.move_to_neutral()

	
	""" Return the endpoint pose of the arm """
	def get_arm(self):
		return self._arm.endpoint_pose()['position']
	
	
	""" Return the endpoint velocity of the arm """
	def get_arm_velocity(self):
		return self._arm.endpoint_velocity()["linear"]


	""" Return the endpoint force of the arm """ 
	def get_arm_force(self):
		return self._arm.endpoint_effort()["force"]
	
	############################################################################
	
	""" 
	If wait is True: check specified position is valid and move the arm to position 
	Otherwise: check to see if position is valid

	"""
	def set_arm(self, pos, rot=(0, math.pi, math.pi *0.5), wait=True):
		resp = self._get_ik(self._arm_iksvc, pos, rot)
		positions = resp[0]
		isValid = resp[1]
		if not isValid:
			print("Invalid position")
		else:
			print("Position is valid")

		if not wait:
			self._arm.set_joint_positions(positions)
		else:
			self._arm.move_to_joint_positions(positions)


	""" Returns specified position with orientation and its validity """
	def _get_ik(self, iksvc, pos, rot):
		q = quaternion_from_euler(rot[0], rot[1], rot[2])

		pose = PoseStamped(
			header=Header(stamp=rospy.Time.now(), frame_id='base'),
			pose=Pose(
				position=Point(
					x=pos[0],
					y=pos[1],
					z=pos[2],
				),
				orientation=Quaternion(q[0], q[1], q[2], q[3])
			),
		)
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(pose)

		iksvc.wait_for_service(5.0)
		resp = iksvc(ikreq)

		positions = dict(zip(resp.joints[0].name, resp.joints[0].position))
		return (positions, resp.isValid[0])
