#!/usr/bin/env python
import rospy
import struct
import math
import baxter_interface
import numpy as np
import transformations as trans
from geometry_msgs.msg import PoseStamped
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


class baxter_controller:
	def __init__(self, limb):
		self.limb = limb
		self.ns = "ExternalTools/" + self.limb + "/PositionKinematicsNode/IKService"	
		self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)


	# robot arm - cursor tracking 	
	def inverse_kinematics(self, command_pose):
		baxter_effector_pose = PoseStamped()
		baxter_effector_pose.header = command_pose.header
		baxter_effector_pose.pose = command_pose.pose
		"""
		baxter_effector_pose.pose.position.x = command_pose.pose.position.x
		baxter_effector_pose.pose.position.y = command_pose.pose.position.y
		baxter_effector_pose.pose.position.z = command_pose.pose.position.z
			
		q_pose = np.array([command_pose.pose.orientation.w, command_pose.pose.orientation.x, command_pose.pose.orientation.y, command_pose.pose.orientation.z])
		q_pose_t = trans.quaternion_conjugate(q_pose)
		
		ori = trans.quaternion_multiply(np.array([1.0/math.sqrt(2.0), 1.0/math.sqrt(2.0), 0, 0]), np.array([1.0/math.sqrt(2.0), 0, 1.0/math.sqrt(2.0), 0]))	
		res_ori = trans.quaternion_multiply(q_pose,ori)

		baxter_effector_pose.pose.orientation.w = res_ori[0]
		baxter_effector_pose.pose.orientation.x = res_ori[1]
		baxter_effector_pose.pose.orientation.y = res_ori[2]
		baxter_effector_pose.pose.orientation.z = res_ori[3]
		"""
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(baxter_effector_pose)

		try:
		    rospy.wait_for_service(self.ns, 0.1)
		    resp = self.iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
		    rospy.logerr("Service call failed: %s" % (e,))
		    return False

		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		
		if (resp_seeds[0] != resp.RESULT_INVALID):
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			self.set_joint_position(limb_joints)
			return True
		else:
			return False

	def set_joint_position(self,limb_joints):
		if self.limb == 'left':
			left = baxter_interface.Limb('left')
			left.set_joint_positions(limb_joints)
		else:
			right = baxter_interface.Limb('right')
			right.set_joint_positions(limb_joints)
