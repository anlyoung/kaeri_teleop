#!/usr/bin/env python


# ROS
import rospy
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import *
from std_msgs.msg import Header
import tf

# python 
import numpy as np
import math

# custom class
import generate_object
#import phantom_visualizer
#import baxter_controller


class command_interface:
	def __init__(self):
		self.command_queue = generate_object.point_queue()
		self.obj_manager = generate_object.rviz_object_manager()
		
		self.cs_pose_world = Pose()
		self.phtm_touched = False

		self.mode = 'config'
		self.action = 'default'

		self.object_moving = False


		self.object_start_pose = Pose()
		self.cylinder_pose = Pose()

		self.x_shift = 0.0
		self.y_shift = 0.0
		self.z_shift = 0.0
		self.radius_increase = 0.0


	## keyboard input
	def vf_action(self, kInput):
		self.action = kInput


	## phantom button command	
	def execute_command(self, bState):
		if self.mode == 'config':
			if bState == 'nothing': self.btnNothing()
			if bState == 'btn1up': self.btn1up(self.cs_pose_world)
			if bState == 'btn1drag': self.btn1drag()
			#if bState == 'btn2down': self.btn2down()
			if bState == 'btn2up': self.btn2up(self.cs_pose_world)
			#if bState == 'btn2drag': self.btn2drag()
				

	def btnNothing(self):
		if self.action == 'default':
			self.command_queue.list = []

			if self.object_moving:
				#transform the object to the last pose
				obj = self.obj_manager.search_object(self.obj_manager.holding_obj.id)
				self.obj_manager.object_list[obj] = self.obj_manager.tf_object(self.object_start_pose, self.cs_pose_world, self.obj_manager.holding_obj)	
				self.obj_manager.render_object()

				#reset saved parameter
				self.obj_manager.holding_obj = generate_object.rviz_object(-1,np.array([]),np.array([]),np.array([]))
				self.object_moving = False
				self.object_start_pose = Pose()


	def btn1up(self, pose):
		if self.action == 'delete':
			if self.phtm_touched:
				position = np.array([self.cs_pose_world.position.x, self.cs_pose_world.position.y, self.cs_pose_world.position.z])
				dist = 10000
				del_id = -1
				for i, obj in enumerate(self.obj_manager.object_list):
					dist_tmp = np.linalg.norm(obj.reference-position)
					if dist_tmp < dist:
						dist = dist_tmp
						del_id = obj.id
				self.obj_manager.delete_object(del_id)
			else:
				self.obj_manager.delete_all()
			#self.obj_manager.render_object()

		if self.action == 'plane':
			self.command_queue.add(pose)
			if len(self.command_queue.list) > 2: 
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []

		if self.action == 'circle':
			self.command_queue.add(pose)
			if len(self.command_queue.list) > 2: 
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []

		if self.action == 'sphere':
			self.command_queue.add(pose)
			if len(self.command_queue.list) > 3: 
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []

		if self.action == 'cylinder':
			self.command_queue.add(pose)
			if len(self.command_queue.list) > 4: 
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []

		if self.action == 'generate_cylinder':
			if len(self.command_queue.list) > 4: 
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []

			else:
				msg = rospy.wait_for_message("/VR_Cylinder_Pos", Pose)

				self.cylinder_pose.position.x = msg.position.z+self.x_shift
				self.cylinder_pose.position.z = msg.position.y/2.0 + self.z_shift
				self.cylinder_pose.position.y = -msg.position.x+ self.y_shift

				self.cylinder_pose.orientation.x = msg.orientation.x*1.3 + self.radius_increase
				self.cylinder_pose.orientation.y = msg.orientation.y/2.0
		
			
				temp = Pose()

				#temp.position = self.cylinder_pose.position
				temp.position.y = self.cylinder_pose.position.y
				temp.position.z = self.cylinder_pose.position.z
				#temp.position.x = temp.position.x - self.cylinder_pose.orientation.x + .2
				temp.position.x +=self.cylinder_pose.position.x +.0001
				temp.position.y += self.cylinder_pose.orientation.x
				temp.position.z +=self.cylinder_pose.orientation.y
				self.command_queue.add(temp)

				temp = Pose()
				#temp.position = self.cylinder_pose.position
				temp.position.y = self.cylinder_pose.position.y
				temp.position.z = self.cylinder_pose.position.z

				#temp.position.x = temp.position.x - self.cylinder_pose.orientation.x + .2
				temp.position.x +=self.cylinder_pose.position.x +.00002
				temp.position.y += self.cylinder_pose.orientation.x	
				temp.position.z -=self.cylinder_pose.orientation.y 
				self.command_queue.add(temp)

				temp = Pose()
				temp.position.y = self.cylinder_pose.position.y
				temp.position.z = self.cylinder_pose.position.z

				#temp.position = self.cylinder_pose.position
				temp.position.x +=self.cylinder_pose.position.x - .0003
				#temp.position.x = temp.position.x - self.cylinder_pose.orientation.x + .2
				temp.position.y -= self.cylinder_pose.orientation.x
				temp.position.z +=self.cylinder_pose.orientation.y
				self.command_queue.add(temp)

				temp = Pose()
				#temp.position = self.cylinder_pose.position
				temp.position.y = self.cylinder_pose.position.y
				temp.position.z = self.cylinder_pose.position.z
				temp.position.x +=self.cylinder_pose.position.x - .00004
				#temp.position.x = temp.position.x - self.cylinder_pose.orientation.x + .2
				temp.position.y -= self.cylinder_pose.orientation.x	
				temp.position.z -=self.cylinder_pose.orientation.y
				self.command_queue.add(temp)

				temp = Pose()
				#temp.position = self.cylinder_pose.position
				temp.position.y = self.cylinder_pose.position.y
				temp.position.z = self.cylinder_pose.position.z

				#temp.position.x = temp.position.x - self.cylinder_pose.orientation.x + .2
				temp.position.x +=self.cylinder_pose.position.x +.0001
				temp.position.y += self.cylinder_pose.orientation.x	
				temp.position.z +=self.cylinder_pose.orientation.y
				self.command_queue.add(temp)

				
				'''
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([temp.position.x,temp.position.y,temp.position.z]))
				obj = self.obj_manager.draw_geometry(self.action, pt_list)
				marker_vf = obj.marker
				marker_pts = self.command_queue.point_marker(pt_list)
				marker_vf.id = 0
				marker_vf.ns = "vf_real_time_update"
				marker_pts.id = 1
				marker_pts.ns = "vf_real_time_update"
				'''

				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []
				self.action = 'default'

		if self.action == 'line':
			self.command_queue.add(pose)

		if self.action == 'polygon':
			self.command_queue.add(pose)


	def btn1drag(self):
		if self.action == 'default':			
			if self.obj_manager.holding_obj.id == -1:					# if nothing was held
				if self.phtm_touched: 										
					position = np.array([self.cs_pose_world.position.x, self.cs_pose_world.position.y, self.cs_pose_world.position.z])
					dist = 10000
					for i, obj in enumerate(self.obj_manager.object_list):
						dist_tmp = np.linalg.norm(obj.reference-position)
						if dist_tmp < dist:
							dist = dist_tmp
							self.obj_manager.holding_obj = obj.copy()

					print(dist_tmp,obj.id,self.obj_manager.holding_obj.id)
					self.object_moving = True
					self.object_start_pose.position.x = self.cs_pose_world.position.x
					self.object_start_pose.position.y = self.cs_pose_world.position.y
					self.object_start_pose.position.z = self.cs_pose_world.position.z
					self.object_start_pose.orientation.x = self.cs_pose_world.orientation.x
					self.object_start_pose.orientation.y = self.cs_pose_world.orientation.y
					self.object_start_pose.orientation.z = self.cs_pose_world.orientation.z
					self.object_start_pose.orientation.w = self.cs_pose_world.orientation.w


	def btn2up(self, pose):
		if self.action == 'line':
			if len(self.command_queue.list) > 1:
				self.command_queue.add(pose)
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []
			else:
				self.command_queue.list = []

		if self.action == 'polygon':
			if len(self.command_queue.list) > 2:
				self.command_queue.add(pose)
				self.obj_manager.add_object(self.obj_manager.draw_geometry(self.action, self.command_queue.list))
				self.command_queue.list = []
			else:
				self.command_queue.list = []


	def real_time_update(self):
		pose = self.cs_pose_world
		markerArray = MarkerArray()
		markerArray.markers = []
		marker_vf = Marker()
		marker_vf.header.frame_id = "world"
		marker_vf.ns = "vf_real_time_update"
		marker_vf.id = 0
		marker_vf.action = Marker.DELETE
		marker_pts = Marker()
		marker_pts.header.frame_id = "world"
		marker_pts.ns = "vf_real_time_update"
		marker_pts.id = 1
		marker_pts.action = Marker.DELETE
		
		if (self.action == 'plane')|(self.action == 'circle'):
			if len(self.command_queue.list) == 0:
				pt_list = []
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				pt_list.append(np.array([pose.position.x+0.1, pose.position.y, pose.position.z]))
				pt_list.append(np.array([pose.position.x, pose.position.y+0.1, pose.position.z]))		
				marker_vf = self.obj_manager.draw_geometry(self.action, pt_list).marker
				marker_pts = self.command_queue.point_marker([np.array([pose.position.x, pose.position.y, pose.position.z])]);
			if len(self.command_queue.list) == 1:
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				pt_list.append(np.array([pose.position.x+0.1, pose.position.y+0.1, pose.position.z+0.1]))				
				marker_vf = self.obj_manager.draw_geometry(self.action, pt_list).marker
				marker_pts = self.command_queue.point_marker([self.command_queue.list[0],np.array([pose.position.x, pose.position.y, pose.position.z])]);
			if len(self.command_queue.list) == 2:
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				marker_vf = self.obj_manager.draw_geometry(self.action, pt_list).marker
				marker_pts = self.command_queue.point_marker(pt_list);
			marker_vf.id = 0
			marker_vf.ns = "vf_real_time_update"
			marker_pts.id = 1
			marker_pts.ns = "vf_real_time_update"

		if (self.action == 'sphere'):
			if len(self.command_queue.list) < 3:
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				marker_pts = self.command_queue.point_marker(pt_list)
				marker_pts.id = 0
				marker_pts.ns = "vf_real_time_update"
			if len(self.command_queue.list) == 3:
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				marker_vf = self.obj_manager.draw_geometry(self.action, pt_list).marker
				marker_pts = self.command_queue.point_marker(pt_list);
				marker_vf.id = 0
				marker_vf.ns = "vf_real_time_update"
				marker_pts.id = 1
				marker_pts.ns = "vf_real_time_update"

		if (self.action == 'cylinder'):
			if len(self.command_queue.list) < 4:
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				marker_pts = self.command_queue.point_marker(pt_list)
				marker_pts.id = 0
				marker_pts.ns = "vf_real_time_update"
			if len(self.command_queue.list) == 4:
				pt_list = list(self.command_queue.list)
				pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
				marker_vf = self.obj_manager.draw_geometry(self.action, pt_list).marker
				marker_pts = self.command_queue.point_marker(pt_list);
				marker_vf.id = 0
				marker_vf.ns = "vf_real_time_update"
				marker_pts.id = 1
				marker_pts.ns = "vf_real_time_update"
		
		if (self.action=='cylinder_up'):
			for i, obj in enumerate(self.obj_manager.object_list):
				del_id = obj.id
				self.obj_manager.delete_object(del_id)

			self.radius_increase +=.01
			self.action = 'generate_cylinder'
			self.btn1up(Pose())

		if (self.action=='cylinder_down'):
			self.obj_manager.delete_all()
			self.radius_increase -=.01
			self.action = 'generate_cylinder'
			self.btn1up(Pose())

		if self.action == 'line':
			pt_list = list(self.command_queue.list)
			pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
			#marker = self.obj_manager.draw_geometry('line', pt_list).marker
			marker_pts = self.command_queue.point_marker(pt_list)
			marker_pts.id = 0
			marker_pts.ns = "vf_real_time_update"
				
		if self.action == 'polygon':
			pt_list = list(self.command_queue.list)
			pt_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
			#marker = self.obj_manager.draw_geometry('polygon', pt_list).marker
			marker_pts = self.command_queue.point_marker(pt_list)
			marker_pts.id = 0
			marker_pts.ns = "vf_real_time_update"

		if self.action == 'default':
			if self.object_moving:
				#obj = self.obj_manager.search_object(self.obj_manager.holding_obj.id)
				marker_vf = self.obj_manager.tf_object(self.object_start_pose, self.cs_pose_world, self.obj_manager.holding_obj).marker
				marker_vf.id = 0
				marker_vf.ns = "vf_real_time_update"

		markerArray.markers.append(marker_vf)
		markerArray.markers.append(marker_pts)
		
		return markerArray				

