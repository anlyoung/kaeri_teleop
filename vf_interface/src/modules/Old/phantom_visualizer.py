#!/usr/bin/env python

import math
import numpy as np
import rospy

from interactive_markers.interactive_marker_server import *
import transformations as trans
import tf
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class phantom_visualizer:
	def __init__(self):
		# cursor pose
		self.cs_pose_phtm = Pose()
		self.cs_pose_world = Pose()
		self.cs_pose_world_initial = Pose()
		self.cs_pose_world_initial.position.x = 0
		self.cs_pose_world_initial.position.y = 0
		self.cs_pose_world_initial.position.z = 0
		
		self.cs_pose_world_initial.orientation.w = 1.0/math.sqrt(2.0)
		self.cs_pose_world_initial.orientation.x = 0
		self.cs_pose_world_initial.orientation.y = 0
		self.cs_pose_world_initial.orientation.z = 1.0/math.sqrt(2.0)

		# tf for frame misalignment between phantom device and openhaptics coordinate
		self.tf_phtm_misalignment = Pose()
		rot1 = trans.quaternion_about_axis(-math.pi/2,[0,1,0])
		rot2 = trans.quaternion_about_axis(math.pi/2,[1,0,0])
		ori_misalignment = trans.quaternion_multiply(rot2,trans.quaternion_multiply(rot1,[1,0,0,0]))
		self.tf_phtm_misalignment.orientation.w = ori_misalignment[0]
		self.tf_phtm_misalignment.orientation.x = ori_misalignment[1]
		self.tf_phtm_misalignment.orientation.y = ori_misalignment[2]
		self.tf_phtm_misalignment.orientation.z = ori_misalignment[3]
		
		# tf for phantom center in rviz virtual space (world)
		self.tf_phtm_rviz = Pose()
		self.tf_phtm_rviz_save = Pose() # for dragging command
		pos = [0.8,0,0.5]
		ori = [1,0,0,0]
		self.tf_phtm_rviz.position.x = pos[0]
		self.tf_phtm_rviz.position.y = pos[1]
		self.tf_phtm_rviz.position.z = pos[2]	
		self.tf_phtm_rviz.orientation.w = ori[0] 
		self.tf_phtm_rviz.orientation.x = ori[1]
		self.tf_phtm_rviz.orientation.y = ori[2]
		self.tf_phtm_rviz.orientation.z = ori[3]
		
		# tf for openhaptics coordinate in rviz virtual space (world)
		self.tf_phtm = self.poseTransform(self.tf_phtm_rviz, self.tf_phtm_misalignment)

		# tf for oculus (IMU reading)
		self.tf_ocls = Pose()
		self.tf_ocls.orientation.w = 1
		
		# tf for misalignment rviz and oculus
		self.tf_ocls_misalignment = Pose()
		self.tf_ocls_misalignment.orientation.w = 1

		# tf for calibrated oculus
		self.tf_ocls_rviz = self.poseTransform(self.tf_ocls, self.tf_ocls_misalignment)
		
		self.button1 = False
		self.button1_save = False
		self.button1_drag = 0
		self.button1_drag_start_pt = Pose()
		self.button1_drag_current_pt = Pose()
		self.button1_drag_end_pt = Pose()
		self.button2 = False
		self.button2_save = False
		self.button2_drag = 0
		self.button2_drag_start_pt = Pose()
		self.button2_drag_current_pt = Pose()
		self.button2_drag_end_pt = Pose()
		self.buttons_save = False
		self.buttons_drag = 0
		self.buttons_drag_start_pt = Pose()
		self.buttons_drag_current_pt = Pose()
		self.buttons_drag_end_pt = Pose()

		self.DRAG_TIME = 10

		self.touched = False
		
		self.cursor = Marker()
		self.cursor.header.frame_id = "world"
		self.cursor.ns = "cursor"
		self.cursor.id = 0;

		self.point = Marker()
		self.point.header.frame_id = "world"
		self.point.ns = "cursor_point"
		self.point.id = 1	
		self.point.type = Marker.POINTS
		self.point.action = Marker.ADD
		self.point.scale.x = 0.01
		self.point.scale.y = 0.01
		self.point.color.r = 1
		self.point.color.g = 1
		self.point.color.b = 1
		self.point.color.a = 1.0


	def poseTransform(self, tf, pose):
		q_pose = np.array([tf.orientation.w, tf.orientation.x, tf.orientation.y, tf.orientation.z])
		q_pose_t = trans.quaternion_conjugate(q_pose)
		pos = np.array([0, pose.position.x, pose.position.y, pose.position.z])
		ori = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
		
		res_pos = trans.quaternion_multiply(q_pose,trans.quaternion_multiply(pos,q_pose_t))
		res_pos = res_pos[1:]
		res_ori = trans.quaternion_multiply(q_pose,ori)

		res_pose = Pose()
		res_pose.position.x = res_pos[0]+tf.position.x
		res_pose.position.y = res_pos[1]+tf.position.y
		res_pose.position.z = res_pos[2]+tf.position.z
		
		res_pose.orientation.w = res_ori[0]
		res_pose.orientation.x = res_ori[1]
		res_pose.orientation.y = res_ori[2]
		res_pose.orientation.z = res_ori[3]
		return res_pose


	def moveCenter(self):
		ori = [self.tf_phtm_rviz.orientation.w, self.tf_phtm_rviz.orientation.x, self.tf_phtm_rviz.orientation.y, self.tf_phtm_rviz.orientation.z]
		direction = np.array([0, self.cs_pose_phtm.position.x + 0.05, self.cs_pose_phtm.position.y, self.cs_pose_phtm.position.z])
		norm = np.linalg.norm(direction) 
		if norm > 0.05:
			direction = direction/np.linalg.norm(direction) 
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply(0.003*direction,trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()


	def rotateCenter(self, start, end):
		ori_s = [start.orientation.w, start.orientation.x, start.orientation.y, start.orientation.z]
		ori_e = [end.orientation.w, end.orientation.x, end.orientation.y, end.orientation.z]
		rot = trans.quaternion_multiply(ori_e,trans.quaternion_conjugate(ori_s))
		res = trans.quaternion_multiply(rot,[self.tf_phtm_rviz_save.orientation.w,self.tf_phtm_rviz_save.orientation.x,self.tf_phtm_rviz_save.orientation.y,self.tf_phtm_rviz_save.orientation.z])
		self.tf_phtm_rviz.orientation.w = res[0]
		self.tf_phtm_rviz.orientation.x = res[1]
		self.tf_phtm_rviz.orientation.y = res[2]
		self.tf_phtm_rviz.orientation.z = res[3]
		self.updateTF()


	def updateTF(self):
		self.tf_phtm = self.poseTransform(self.tf_phtm_rviz, self.tf_phtm_misalignment)


	def updateCursor(self, mode):
		if mode == "config":
			self.cursor.type = Marker.SPHERE
			self.cursor.action = Marker.ADD
			self.cursor.scale.x = 0.03      	
			self.cursor.scale.y = 0.03			
			self.cursor.scale.z = 0.03      		
			self.cursor.color.a = 1.0
		else:
			"""
			self.cursor.type = Marker.ARROW
			self.cursor.action = Marker.ADD
			self.cursor.scale.x = 0.01      	# shaft diameter
			self.cursor.scale.y = 0.03				# head diameter
			self.cursor.scale.z = 0.1      		# head length
			self.cursor.points = []
			self.cursor.points.append(Point(-0.2, 0.0, 0.0))
			self.cursor.points.append(Point(0.00, 0.0, 0.0))
			"""
			self.cursor.type = Marker.TRIANGLE_LIST
			self.cursor.action = Marker.ADD
			self.cursor.scale.x = 1.0
			self.cursor.scale.y = 1.0
			self.cursor.scale.z = 1.0
			self.cursor.color.a = 1.0
			self.cursor.points.append(Point(0.1, 0.0, 0.0))
			self.cursor.points.append(Point(0.0, 0.0, 0.0))
			self.cursor.points.append(Point(0.0, -0.01, 0.0))
			self.cursor.points.append(Point(0.0, 0.2, 0.0))
			self.cursor.points.append(Point(0.0, 0.0, 0.0))
			self.cursor.points.append(Point(0.01, 0.0, 0.0))
			self.cursor.points.append(Point(0.0, 0.0, 0.3))
			self.cursor.points.append(Point(0.0, 0.0, 0.0))
			self.cursor.points.append(Point(0.0, -0.01, 0.0))

		self.cursor.pose = self.cs_pose_world
		return self.cursor


	def colorCursor(self, color):
		if color == "Yellow":
			self.cursor.color.r = 1
			self.cursor.color.g = 1
			self.cursor.color.b = 0
		elif color == "Green":
			self.cursor.color.r = 0
			self.cursor.color.g = 1
			self.cursor.color.b = 0
		elif color == "Red":
			self.cursor.color.r = 1
			self.cursor.color.g = 0
			self.cursor.color.b = 0

	
	def addPoint(self, pose):
		self.point.points.append(Point(pose.position.x, pose.position.y, pose.position.z))
		return self.point


	def clearPoints(self):
		self.point.points = []
		return self.point		


	def buttonState(self):
		# (buttons down both) or (any button stays down after both buttons down)
		if (self.button1 and self.button2) or (self.button1 and self.buttons_save) or (self.button2 and self.buttons_save):	
			self.button1_save = False
			self.button1_drag = 0
			self.button2_save = False
			self.button2_drag = 0
			if self.buttons_save:			
				self.buttons_drag = self.buttons_drag+1
				if self.buttons_drag > self.DRAG_TIME:
					self.buttons_drag_current_pt.position = self.cs_pose_phtm.position
					return "btnsdrag"		# buttons dragging
				else:
					return "nothing"		# wait for dragging
			else:							
				self.buttons_save = True
				self.buttons_drag_start_pt.position = self.cs_pose_phtm.position
				return "btnsdown"			# buttons just down

		# check button release (no button down)
		if self.buttons_save:
			self.buttons_save = False	
			# save end point if dragging and reset start point
			if self.buttons_drag > self.DRAG_TIME:
				self.buttons_drag_end_pt.position = self.cs_pose_phtm.position
			self.buttons_drag_start_pt = Pose()
			self.buttons_drag = 0
			return "btnsup"					# buttons just released

		# button 1 down
		if self.button1: 					
			if self.button1_save: 			
				self.button1_drag = self.button1_drag+1
				if self.button1_drag > self.DRAG_TIME:
					self.button1_drag_current_pt = self.cs_pose_phtm
					return "btn1drag"		# button1 dragging
				else:
					return "nothing"		# wait for dragging
			else: 							# button1 just down
				self.button1_save = True
				self.button1_drag_start_pt = self.cs_pose_phtm
				return "btn1down"

		# button 2 down
		if self.button2: 					
			if self.button2_save: 			
				self.button2_drag = self.button2_drag+1
				if self.button2_drag > self.DRAG_TIME:
					self.button2_drag_current_pt = self.cs_pose_phtm
					return "btn2drag"		# button2 dragging
				else:
					return "nothing"		# wait for dragging
			else: 							
				self.button2_save = True
				self.button2_drag_start_pt = self.cs_pose_phtm
				return "btn2down"			# button2 just down

		# button 1 released
		if self.button1_save:
			if self.button1_drag > self.DRAG_TIME:
				self.button1_drag_end_pt = self.cs_pose_phtm
				self.button1_drag_start_pt = Pose()
				self.button1_drag = 0
				self.button1_save = False		
				return "nothing"
			else:
				self.button1_drag_start_pt = Pose()
				self.button1_drag = 0
				self.button1_save = False				
				return "btn1up"					# button1 just released

		# button 2 released
		if self.button2_save:
			if self.button2_drag > self.DRAG_TIME:
				self.button2_drag_end_pt = self.cs_pose_phtm
				self.button2_drag_start_pt = Pose()
				self.button2_drag = 0
				self.button2_save = False
				return "nothing"
			else:
				self.button2_drag_start_pt = Pose()
				self.button2_drag = 0
				self.button2_save = False
				return "btn2up"					# button2 just released

		# nothing happened
		return "nothing"

		# double click can be added later


	"""
	def phtmCoordinateSync(self,pos):
		rot1 = trans.quaternion_about_axis(-math.pi/2,[0,1,0])
		rot2 = trans.quaternion_about_axis(math.pi/2,[1,0,0])
		ori = trans.quaternion_multiply(rot2,trans.quaternion_multiply(rot1,[1,0,0,0]))
		res_pos = trans.quaternion_multiply(ori,trans.quaternion_multiply([0, pos.x, pos.y, pos.z],trans.quaternion_conjugate(ori)))
		res_pos = res_pos[1:]
		return res_pos
	"""
	"""
	def moveCenter(self, direction):		
		if direction == 'forward':
			ori = [self.tf_ocls_rviz.orientation.w,self.tf_ocls_rviz.orientation.x,self.tf_ocls_rviz.orientation.y,self.tf_ocls_rviz.orientation.z]
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply([0,0.05,0,0],trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()
		if direction == 'backward':
			ori = [self.tf_ocls_rviz.orientation.w,self.tf_ocls_rviz.orientation.x,self.tf_ocls_rviz.orientation.y,self.tf_ocls_rviz.orientation.z]
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply([0,-0.05,0,0],trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()
		if direction == 'left':
			ori = [self.tf_ocls_rviz.orientation.w,self.tf_ocls_rviz.orientation.x,self.tf_ocls_rviz.orientation.y,self.tf_ocls_rviz.orientation.z]
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply([0,0,0.05,0],trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()
	"""
	"""
			rot = trans.quaternion_about_axis(math.pi/180,[0,0,1])
			ori = [self.tf_phtm_rviz.orientation.w,self.tf_phtm_rviz.orientation.x,self.tf_phtm_rviz.orientation.y,self.tf_phtm_rviz.orientation.z]
			res = trans.quaternion_multiply(rot,ori)
			self.tf_phtm_rviz.orientation.w = res[0]
			self.tf_phtm_rviz.orientation.x = res[1]
			self.tf_phtm_rviz.orientation.y = res[2]
			self.tf_phtm_rviz.orientation.z = res[3]
			self.updateTF()
	"""			
	"""
		if direction == 'right':
			ori = [self.tf_ocls_rviz.orientation.w,self.tf_ocls_rviz.orientation.x,self.tf_ocls_rviz.orientation.y,self.tf_ocls_rviz.orientation.z]
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply([0,0,-0.05,0],trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()
		if direction == 'up':
			ori = [self.tf_ocls_rviz.orientation.w,self.tf_ocls_rviz.orientation.x,self.tf_ocls_rviz.orientation.y,self.tf_ocls_rviz.orientation.z]
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply([0,0,0,0.05],trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()
		if direction == 'down':
			ori = [self.tf_ocls_rviz.orientation.w,self.tf_ocls_rviz.orientation.x,self.tf_ocls_rviz.orientation.y,self.tf_ocls_rviz.orientation.z]
			movement = trans.quaternion_multiply(ori,trans.quaternion_multiply([0,0,0,-0.05],trans.quaternion_conjugate(ori)))
			self.tf_phtm_rviz.position.x = self.tf_phtm_rviz.position.x + movement[1]
			self.tf_phtm_rviz.position.y = self.tf_phtm_rviz.position.y + movement[2]
			self.tf_phtm_rviz.position.z = self.tf_phtm_rviz.position.z + movement[3]
			self.updateTF()
	

	def rotateCenter(self, startPt, endPt):
		Start = [0, startPt.position.x, startPt.position.y, startPt.position.z]
		End = [0, endPt.position.x, endPt.position.y, endPt.position.z]
		nStart = trans.unit_vector(Start)
		nEnd = trans.unit_vector(End)
		rot = trans.quaternion_multiply(nEnd,trans.quaternion_conjugate(nStart))
		res = trans.quaternion_multiply(rot,[self.tf_phtm_rviz_save.orientation.w,self.tf_phtm_rviz_save.orientation.x,self.tf_phtm_rviz_save.orientation.y,self.tf_phtm_rviz_save.orientation.z])
		self.tf_phtm_rviz.orientation.w = res[0]
		self.tf_phtm_rviz.orientation.x = res[1]
		self.tf_phtm_rviz.orientation.y = res[2]
		self.tf_phtm_rviz.orientation.z = res[3]
		self.updateTF()
	"""

