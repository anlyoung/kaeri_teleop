#!/usr/bin/env python
# modules for stl file delete
import os
import glob

import rospy

# others
import math
import numpy as np
#import stl
import transformations as trans
#from stl import mesh

from visualization_msgs.msg import *
#from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from std_msgs.msg import Float32MultiArray

class point_queue:
	def __init__(self):
		self.list = []

	def add(self, pose):
		pt = np.array([pose.position.x, pose.position.y, pose.position.z])
		self.list.append(pt)

	def point_marker(self, pts_list):
		marker = Marker()
		marker.header.frame_id = "world"
		marker.ns = ""
		marker.id = -1
		marker.type = Marker.POINTS
		marker.action = Marker.ADD
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		for i in range(len(pts_list)):
			marker.points.append(Point(pts_list[i][0],pts_list[i][1],pts_list[i][2]))

		return marker
 

################################################################################
class rviz_object:
	def __init__(self, object_id, vtx, face, norm):
		self.id = object_id
		self.vertex = np.copy(vtx)
		self.face = np.copy(face)
		self.norm = np.copy(norm)
		
		self.marker = self.create_marker()

		
		# reference point for scale change
		self.reference = np.zeros((3,1))
		if self.vertex.shape[0] > 2:
			for i in range(3): self.reference[i] = np.average(self.vertex[i,:])

	def set_id(self, obj_id):
		self.id = obj_id
		self.marker.id = obj_id

	def create_marker(self):
		marker = Marker()
		marker.header.frame_id = "world"
		marker.ns = "vf_mesh"
		marker.id = self.id
		#print(marker.id)		

		#marker.type = Marker.MESH_RESOURCE
		#marker.mesh_resource = "package://rviz_controller/objects/" + "bin_%d" %self.id + ".stl"
		marker.type = Marker.TRIANGLE_LIST		
		marker.action = Marker.ADD
		marker.scale.x = 1
		marker.scale.y = 1
		marker.scale.z = 1
		marker.color.r = 0.5
		marker.color.g = 0.5
		marker.color.b = 0.5
		marker.color.a = 0.5
		#marker.color.a = 1
		
		vtx = np.transpose(self.vertex)
		fc = np.transpose(self.face)

			#fc_permute = np.transpose([fc[:,0], fc[:,2], fc[:,1]])
			#fc = np.concatenate((fc, fc_permute), axis=0)
		for i, f in enumerate(fc):
			for j in range(3):
				p = Point()
				p.x = vtx[int(f[j]),0]
				p.y = vtx[int(f[j]),1]
				p.z = vtx[int(f[j]),2]
				marker.points.append(p)

		marker.pose = Pose()
		marker.pose.orientation.w = 1
		return marker


	def delete_marker(self):
		marker = Marker()
		marker.header.frame_id = "world"
		marker.ns = "vf_mesh"
		marker.id = self.id
		marker.action = Marker.DELETE
		
		return marker

	
	def copy(self):
		return rviz_object(self.id, self.vertex, self.face, self.norm)
		

################################################################################
class rviz_object_manager:
	def __init__(self):
		path = os.path.dirname(os.path.realpath(__file__))
		path, dum = os.path.split(path)
		path, dum = os.path.split(path)
		print(path)
		self.path = path
		
		self.obj_path = os.path.join(self.path,'objects')
		self.object_list = []
		self.markerArray = MarkerArray()
		self.object_id = 0

		self.holding_obj = rviz_object(-1, np.array([]), np.array([]), np.array([]))

		# remove all existing files in STL folder
		STLS = glob.glob(os.path.join(self.obj_path,'*.stl'))
		for f in STLS:
			os.remove(f)


	def check_update(self):
		if self.update:
			self.update = False
			return True
		else:
			return False


	def add_object(self, obj, show = True):
		# generate object
		obj.set_id(self.object_id)
		self.object_list.append(obj)
		if show: self.render_object()
		self.object_id = self.object_id+1
		for i in range(len(self.markerArray.markers)):
			print(self.markerArray.markers[i].id)


	def tf_object(self, start, end, obj):
		translation = [end.position.x - start.position.x, end.position.y - start.position.y, end.position.z - start.position.z] 
		
		ori_s = [start.orientation.w, start.orientation.x, start.orientation.y, start.orientation.z]
		ori_e = [end.orientation.w, end.orientation.x, end.orientation.y, end.orientation.z]
		rot = trans.quaternion_multiply(ori_e,trans.quaternion_conjugate(ori_s))
		rotation = trans.quaternion_matrix(rot)
		
		vertex = np.vstack((obj.vertex, np.ones(obj.vertex.shape[1])))
		#print(obj.reference)
		vertex = vertex - np.array([obj.reference[0]*np.ones(vertex.shape[1]), obj.reference[1]*np.ones(vertex.shape[1]), obj.reference[2]*np.ones(vertex.shape[1]), np.zeros(vertex.shape[1])])

		vertex = np.dot(rotation,vertex)
		vertex = vertex[:3,:]

		vertex = vertex + np.array([obj.reference[0]*np.ones(vertex.shape[1]), obj.reference[1]*np.ones(vertex.shape[1]), obj.reference[2]*np.ones(vertex.shape[1])])
	
		vertex = vertex + np.array([translation[0]*np.ones(vertex.shape[1]), translation[1]*np.ones(vertex.shape[1]), translation[2]*np.ones(vertex.shape[1])])
		
		obj = rviz_object(obj.id, vertex, obj.face, obj.norm)
		return obj


	def delete_object(self, obj_id):
		self.markerArray.markers = []
		new_obj_list = []
		for i, marker in enumerate(self.object_list):
			if marker.id is not obj_id:
				self.markerArray.markers.append(marker.marker)
				new_obj_list.append(marker)
			else:
				self.markerArray.markers.append(marker.delete_marker())

		self.object_list = new_obj_list


	def delete_all(self):
		self.markerArray.markers = []
		for i, marker in enumerate(self.object_list):
			self.markerArray.markers.append(marker.delete_marker())
		self.object_list = []
		
		# remove all existing files in STL folder
		#STLS = glob.glob(os.path.join(self.obj_path,'*.stl'))
		#for f in STLS:
		#	os.remove(f)
		
		# update haptic
		#self.update = True


	def search_object(self, obj_id):
		#target = rviz_object(-1, np.array([]), np.array([]), np.array([]))
		idx = -1
		for i, marker in enumerate(self.object_list):
			if marker.id == obj_id:
				idx = i
		return i

	
	def render_object(self):
		self.markerArray.markers = []
		for i, obj in enumerate(self.object_list):
			self.markerArray.markers.append(obj.marker)


	def draw_geometry(self, geometry_type, pts_list):
		obj = rviz_object(-1, np.array([]), np.array([]), np.array([]))
		if geometry_type == 'plane':
			obj = self.draw_plane(pts_list)

		if geometry_type == 'line':
			obj = self.draw_line(pts_list)

		if geometry_type == 'circle':
			obj = self.draw_circle(pts_list)

		if geometry_type == 'sphere':
			obj = self.draw_sphere(pts_list)

		if geometry_type == 'cylinder':
			obj = self.draw_cylinder(pts_list)

		if geometry_type == 'generate_cylinder':
			obj = self.draw_generated_cylinder(pts_list)

		if geometry_type == 'polygon':
			obj = self.draw_polygon(pts_list)

		return obj


	def draw_plane(self, pts_list):
		pt0 = pts_list[0]
		pt1 = pts_list[1]
		pt2 = pts_list[2]

		center = (pt0 + pt1 + pt2)/3.0
		
		v0 = pt1 - pt0
		v1 = pt2 - pt0

		norm = np.cross(v0, v1)
		norm_length = np.linalg.norm(norm)
		if norm_length > 0: norm = norm/norm_length

		vertex = np.zeros((3,4))
		face = np.zeros((3,2))
		vertex[0,0] = 0.5/4
		vertex[1,0] = 0.5/4
		vertex[2,0] = 0.0
		vertex[0,1] = -0.5/4
		vertex[1,1] = 0.5/4
		vertex[2,1] = 0.0
		vertex[0,2] = -0.5/4
		vertex[1,2] = -0.5/4
		vertex[2,2] = 0.0
		vertex[0,3] = 0.5/4
		vertex[1,3] = -0.5/4
		vertex[2,3] = 0.0
		
		face[0,0] = 0
		face[1,0] = 1
		face[2,0] = 2
		face[0,1] = 2
		face[1,1] = 3
		face[2,1] = 0

		# add a row of 1s for rotation matrix multiply				
		vertex = np.vstack((vertex, np.ones((1,4))))

		# transform
		rotation = self.plane_transform(norm)
		vertex = np.dot(rotation,vertex)
		vertex = vertex[:3,:]
		vertex = vertex + np.array([center[0]*np.ones(vertex.shape[1]), center[1]*np.ones(vertex.shape[1]), center[2]*np.ones(vertex.shape[1])])
	
		obj = rviz_object(-1, vertex, face, norm)
		return obj


	def draw_line(self, pts_list):
		length = 0.1			# length of triangular guide surface		

		dir_pt = pts_list[-1]
		nPts = len(pts_list)-1		
				
		####### start of modified code #######
		cmass = np.zeros((3))
		for i in range(nPts):
			for j in range(3):
				cmass[j] = cmass[j] + pts_list[i][j]
		cmass = cmass/nPts

		v_n = dir_pt - cmass
		#print(v_n)
		v_n = v_n/np.linalg.norm(v_n)
		#print(v_n)

		vertex = np.zeros((3, 2*nPts))
		face = np.zeros((3, 2*(nPts-1)))

		# for the first point of the line
		vec = pts_list[1] - pts_list[0]
		#v_n = self.find_normal(vec, pts_list[0], dir_pt)
		for j in range(3):
			vertex[j,0] = pts_list[0][j] - length*v_n[j]
			vertex[j,1] = pts_list[0][j] + length*v_n[j]

		# for the 2 ~ nPts-1 point of the line
		if nPts > 2:
			for i in range(nPts-2):
				vec1 = pts_list[i+1] - pts_list[i]
				vec2 = pts_list[i+2] - pts_list[i+1]

				vec1 = vec1/np.linalg.norm(vec1)
				vec2 = vec2/np.linalg.norm(vec2)
			
				#v_n1 = self.find_normal(vec1, pts_list[i+1], dir_pt)
				#v_n2 = self.find_normal(vec2, pts_list[i+1], dir_pt)

				#res = (v_n1 + v_n2)/2
			
				#res = res/np.linalg.norm(res)

				#length_n = 1/math.sqrt(1 - pow(vec1.dot(res),2))
				#print(length1, length2)

				for j in range(3):			
					vertex[j,2*(i+1)] = pts_list[i+1][j] - length*v_n[j]
					#vertex[j,2*(i+1)+1] = pts_list[i+1][j] + length*length_n*res[j]
					vertex[j,2*(i+1)+1] = pts_list[i+1][j] + length*v_n[j]

		# for the last point of the line
		vec = pts_list[nPts-1] - pts_list[nPts-2]
		#v_n = self.find_normal(vec, pts_list[nPts-1], dir_pt)
		for j in range(3):
			vertex[j,2*nPts-2] = pts_list[nPts-1][j] - length*v_n[j]
			vertex[j,2*nPts-1] = pts_list[nPts-1][j] + length*v_n[j]

		# face assignment
		for i in range(nPts-1):
			face[0,2*i] = 2*i
			face[1,2*i] = 2*i+1
			face[2,2*i] = 2*i+3

			face[0,2*i+1] = 2*i+3
			face[1,2*i+1] = 2*i+2
			face[2,2*i+1] = 2*i

		"""
		cmass = np.zeros((3))
		for i in range(nPts):
			for j in range(3):
				cmass[j] = cmass[j] + pts_list[i][j]
		cmass = cmass/nPts

		vertex = np.zeros((3, 3*nPts))
		face = np.zeros((3, 4*(nPts-1)))

		# for the first point of the line
		vec = pts_list[1] - pts_list[0]
		#vecs = self.triangular_normal(vec, pts_list[0], dir_pt)
		vecs = self.triangular_normal(vec, cmass, dir_pt)
		for j in range(3):
			vertex[j,0] = pts_list[0][j]
			vertex[j,1] = pts_list[0][j] + length*vecs[0][j]
			vertex[j,2] = pts_list[0][j] + length*vecs[1][j]

		# for the 2 ~ nPts-1 point of the line
		if nPts > 2:
			for i in range(nPts-2):
				vec1 = pts_list[i+1] - pts_list[i]
				vec2 = pts_list[i+2] - pts_list[i+1]

				vec1 = vec1/np.linalg.norm(vec1)
				vec2 = vec2/np.linalg.norm(vec2)
			
				#vecs1 = self.triangular_normal(vec1, pts_list[i+1], dir_pt)
				#vecs2 = self.triangular_normal(vec2, pts_list[i+1], dir_pt)
				vecs1 = self.triangular_normal(vec1, cmass, dir_pt)
				vecs2 = self.triangular_normal(vec2, cmass, dir_pt)

				res1 = (vecs1[0] + vecs2[0])/2
				res2 = (vecs1[1] + vecs2[1])/2
			
				res1 = res1/np.linalg.norm(res1)
				res2 = res2/np.linalg.norm(res2)

				length1 = 1/math.sqrt(1 - pow(vec1.dot(res1),2))
				length2 = 1/math.sqrt(1 - pow(vec1.dot(res2),2))
				#print(length1, length2)

				for j in range(3):			
					vertex[j,3*(i+1)] = pts_list[i+1][j]
					vertex[j,3*(i+1)+1] = pts_list[i+1][j] + length*length1*res1[j]
					vertex[j,3*(i+1)+2] = pts_list[i+1][j] + length*length2*res2[j]

		# for the last point of the line
		vec = pts_list[nPts-1] - pts_list[nPts-2]
		#vecs = self.triangular_normal(vec, pts_list[nPts-1], dir_pt)
		vecs = self.triangular_normal(vec, cmass, dir_pt)
		for j in range(3):
			vertex[j,3*nPts-3] = pts_list[nPts-1][j]
			vertex[j,3*nPts-2] = pts_list[nPts-1][j] + length*vecs[0][j]
			vertex[j,3*nPts-1] = pts_list[nPts-1][j] + length*vecs[1][j]

		# face assignment
		for i in range(nPts-1):
			face[0,4*i] = 3*i
			face[1,4*i] = 3*i+1
			face[2,4*i] = 3*i+3

			face[0,4*i+1] = 3*i+4
			face[1,4*i+1] = 3*i+3
			face[2,4*i+1] = 3*i+1

			face[0,4*i+2] = 3*i
			face[1,4*i+2] = 3*i+3
			face[2,4*i+2] = 3*i+2
		
			face[0,4*i+3] = 3*i+5
			face[1,4*i+3] = 3*i+2
			face[2,4*i+3] = 3*i+3
		"""

		normal = np.array([]) 		# no normal calculation (not need yet)

		obj = rviz_object(-1, vertex, face, normal)
		return obj


	def draw_circle(self, pts_list, nvtx = 40):
		center = pts_list[0] 
		point1 = pts_list[1]
		point2 = pts_list[2]

		r = np.linalg.norm(point1-center)
		x1 = point1-center
		x2 = point2-center
		norm = np.cross(x1, x2)
		#print(norm)
		norm = norm/np.linalg.norm(norm)
		#print(norm)

		# generate vertice and faces of bottom circle
		vertex1 = np.zeros((3,nvtx+1))
		face1 = np.zeros((3,nvtx))
		face1[0,:] = np.zeros((1,nvtx))
		for i in range(nvtx):
			vertex1[0,i+1] = r*math.cos(2*math.pi/nvtx*i)
			vertex1[1,i+1] = r*math.sin(2*math.pi/nvtx*i)
			vertex1[2,i+1] = 0
			if i == nvtx-1:			# faces (center, i, i+1)
				face1[1,i] = nvtx
				face1[2,i] = 1
				
			else:					# last face (center, nvtx, 1)
				face1[1,i] = i+1
				face1[2,i] = i+2	

		# generate vertice and faces of side wall
		vertex2 = np.zeros((3,nvtx+1))
		face2 = np.zeros((3,nvtx*2))
		for i in range(nvtx):
			vertex2[0,i+1] = r*1.5*math.cos(2*math.pi/nvtx*i)
			vertex2[1,i+1] = r*1.5*math.sin(2*math.pi/nvtx*i)
			vertex2[2,i+1] = 0.1
			if i == nvtx-1:			# faces (center, i, i+1)
				face2[0,2*i] = i+1
				face2[1,2*i] = (nvtx+1)+i+1
				face2[2,2*i] = 1

				face2[0,2*i+1] = 1
				face2[1,2*i+1] = (nvtx+1)+i+1
				face2[2,2*i+1] = (nvtx+1)+1
				
			else:					# last face (center, nvtx, 1)
				face2[0,2*i] = i+1
				face2[1,2*i] = (nvtx+1)+i+1
				face2[2,2*i] = i+2

				face2[0,2*i+1] = i+2
				face2[1,2*i+1] = (nvtx+1)+i+1
				face2[2,2*i+1] = (nvtx+1)+i+2	

		vertex = np.hstack((vertex1,vertex2))
		face = np.hstack((face1,face2))

		# add a row of 1s for rotation matrix multiply				
		vertex = np.vstack((vertex, np.ones((1,vertex.shape[1]))))
		#print(vertex)
		# calculate rotation matrix
		axis = np.cross(norm,np.array([0,0,1]))
		
		ang_mat = np.array([axis,norm,np.array([0,0,1])])
		#print(ang_mat)
		if np.linalg.det(ang_mat) > 0:
			angle = -np.arccos(np.dot(norm,np.array([0,0,1])))
		else:
			angle = np.arccos(np.dot(norm,np.array([0,0,1])))
		
		quaternion = trans.quaternion_about_axis(angle,axis)
		rotation = trans.quaternion_matrix(quaternion)
	
		# transform
		vertex = np.dot(rotation,vertex)
		vertex = vertex[:3,:]
		vertex = vertex + np.array([center[0]*np.ones(vertex.shape[1]),center[1]*np.ones(vertex.shape[1]),center[2]*np.ones(vertex.shape[1])])
		
		# generate normals of each face
		normal = np.zeros((3,face.shape[1]))
		#normal = np.tile(norm,(1,nvtx))
	
		# generate object
		obj = rviz_object(-1, vertex, face, normal)

		return obj


	def draw_sphere(self, pts_list, nvtx = 20):
		# 4 points in pts_list
		x0 = pts_list[0]-pts_list[1]
		x1 = pts_list[1]-pts_list[2]
		x2 = pts_list[2]-pts_list[3]
		
		p0_n = np.linalg.norm(pts_list[0])
		p1_n = np.linalg.norm(pts_list[1])
		p2_n = np.linalg.norm(pts_list[2])
		p3_n = np.linalg.norm(pts_list[3])

		A = np.matrix([x0,x1,x2])
		b = np.array([(p0_n**2-p1_n**2)/2,(p1_n**2-p2_n**2)/2,(p2_n**2-p3_n**2)/2])
		if np.linalg.matrix_rank(A) == 3:
			x = np.dot(np.linalg.inv(A),b)
			r = np.linalg.norm(x-pts_list[0])
			
			vertex = np.zeros((3,(nvtx+1)*nvtx))
			face = np.zeros((3,2*nvtx*nvtx))

			# use spherical coordinate system (2*nvtx points along each axis)
			for i in range(nvtx+1):
				for j in range(nvtx):
					vertex[0,i*nvtx+j] = x[0,0]+r*math.sin(math.pi/nvtx*i)*math.cos(2*math.pi/nvtx*j)
					vertex[1,i*nvtx+j] = x[0,1]+r*math.sin(math.pi/nvtx*i)*math.sin(2*math.pi/nvtx*j)
					vertex[2,i*nvtx+j] = x[0,2]+r*math.cos(math.pi/nvtx*i)

					if (i<nvtx) & (j<nvtx-1):
						face[0,i*nvtx+j] = i*nvtx+j
						face[1,i*nvtx+j] = (i+1)*nvtx+j
						face[2,i*nvtx+j] = (i+1)*nvtx+j+1
								
						face[0,nvtx*nvtx+i*nvtx+j] = (i+1)*nvtx+j+1
						face[1,nvtx*nvtx+i*nvtx+j] = i*nvtx+j+1
						face[2,nvtx*nvtx+i*nvtx+j] = i*nvtx+j
					
					elif (i<nvtx) & (j==nvtx-1):
						face[0,i*nvtx+j] = i*nvtx+j
						face[1,i*nvtx+j] = (i+1)*nvtx+j
						face[2,i*nvtx+j] = (i+1)*nvtx
								
						face[0,nvtx*nvtx+i*nvtx+j] = (i+1)*nvtx
						face[1,nvtx*nvtx+i*nvtx+j] = i*nvtx
						face[2,nvtx*nvtx+i*nvtx+j] = i*nvtx+j	

		else:
			vertex = np.zeros((3,1))
			face = np.zeros((3,1))

		normal = np.array([]) 		# no normal calculation (not need yet)
		obj = rviz_object(-1, vertex, face, normal)
		return obj			
			

	def draw_cylinder(self, pts_list, nvtx = 20):
		l = pts_list[1] - pts_list[0]
		length = np.linalg.norm(l)
		l_n  = l/length
		mp = (pts_list[1]+pts_list[0])/2			# mid point

		v0 = pts_list[2] - mp
		v1 = pts_list[3] - mp
		v2 = pts_list[4] - mp

		# projection of pt2, pt3, pt4 on the perpendicular plane of l
		v0_p = v0 - l_n*np.dot(l_n,v0)
		v1_p = v1 - l_n*np.dot(l_n,v1)
		v2_p = v2 - l_n*np.dot(l_n,v2)
		
		v0_n = np.linalg.norm(v0_p)
		v1_n = np.linalg.norm(v1_p)
		v2_n = np.linalg.norm(v2_p)		

		a0 = v0_p - v1_p
		a1 = v1_p - v2_p
	
		# equation of circle
		A = np.matrix([a0,a1,l_n])
		b = np.array([(v0_n**2-v1_n**2)/2,(v1_n**2-v2_n**2)/2,0])

		if np.linalg.matrix_rank(A) == 3:
			# calculate center and radius
			x = np.dot(np.linalg.inv(A),b)
			r = np.linalg.norm(x-v0_p)
			
			vertex = np.zeros((3,2*nvtx))
			face = np.zeros((3,2*nvtx))

			# use cylinderical coordinate system (nvtx points along circle curve)
			for i in range(nvtx):
				vertex[0,i] = r*math.cos(2*math.pi/nvtx*i)
				vertex[1,i] = r*math.sin(2*math.pi/nvtx*i)
				vertex[2,i] = -length/2

				vertex[0,nvtx+i] = r*math.cos(2*math.pi/nvtx*i)
				vertex[1,nvtx+i] = r*math.sin(2*math.pi/nvtx*i)
				vertex[2,nvtx+i] = length/2
				
				if i < nvtx-1:				
					face[0,i] = i
					face[1,i] = i+1
					face[2,i] = nvtx+i+1
							
					face[0,nvtx+i] = nvtx+i+1
					face[1,nvtx+i] = nvtx+i
					face[2,nvtx+i] = i
				else:
					face[0,i] = i
					face[1,i] = 0
					face[2,i] = nvtx
							
					face[0,nvtx+i] = nvtx
					face[1,nvtx+i] = nvtx+i
					face[2,nvtx+i] = i

			# transformation from origin to mp
			vertex = np.vstack((vertex, np.ones((1,vertex.shape[1]))))

			axis = np.cross(l_n,np.array([0,0,1]))		
			ang_mat = np.array([axis,l_n,np.array([0,0,1])])

			if np.linalg.det(ang_mat) > 0:
				angle = -np.arccos(np.dot(l_n,np.array([0,0,1])))
			else:
				angle = np.arccos(np.dot(l_n,np.array([0,0,1])))
		
			quaternion = trans.quaternion_about_axis(angle,axis)
			rotation = trans.quaternion_matrix(quaternion)
	
			vertex = np.dot(rotation,vertex)
			vertex = vertex[:3,:]
			vertex = vertex + np.array([(mp[0]+x[0,0])*np.ones(vertex.shape[1]),(mp[1]+x[0,1])*np.ones(vertex.shape[1]),(mp[2]+x[0,2])*np.ones(vertex.shape[1])])

		else:
			vertex = np.zeros((3,1))
			face = np.zeros((3,1))

		normal = np.array([]) 		# no normal calculation (not need yet)
		obj = rviz_object(-1, vertex, face, normal)
		return obj	

	def draw_generated_cylinder(self,pts_list, nvtx = 20):
		l = pts_list[1] - pts_list[0]
		length = np.linalg.norm(l)
		l_n  = l/length
		mp = (pts_list[1]+pts_list[0])/2			# mid point

		v0 = pts_list[2] - mp
		v1 = pts_list[3] - mp
		v2 = pts_list[4] - mp

		# projection of pt2, pt3, pt4 on the perpendicular plane of l
		v0_p = v0 - l_n*np.dot(l_n,v0)
		v1_p = v1 - l_n*np.dot(l_n,v1)
		v2_p = v2 - l_n*np.dot(l_n,v2)
		
		v0_n = np.linalg.norm(v0_p)
		v1_n = np.linalg.norm(v1_p)
		v2_n = np.linalg.norm(v2_p)		

		a0 = v0_p - v1_p
		a1 = v1_p - v2_p
	
		# equation of circle
		A = np.matrix([a0,a1,l_n])
		b = np.array([(v0_n**2-v1_n**2)/2,(v1_n**2-v2_n**2)/2,0])

		if np.linalg.matrix_rank(A) == 3:
			# calculate center and radius
			x = np.dot(np.linalg.inv(A),b)
			r = np.linalg.norm(x-v0_p)
			
			vertex = np.zeros((3,2*nvtx))
			face = np.zeros((3,2*nvtx))

			# use cylinderical coordinate system (nvtx points along circle curve)
			for i in range(nvtx):
				vertex[0,i] = r*math.cos(2*math.pi/nvtx*i)
				vertex[1,i] = r*math.sin(2*math.pi/nvtx*i)
				vertex[2,i] = -length/2

				vertex[0,nvtx+i] = r*math.cos(2*math.pi/nvtx*i)
				vertex[1,nvtx+i] = r*math.sin(2*math.pi/nvtx*i)
				vertex[2,nvtx+i] = length/2
				
				if i < nvtx-1:				
					face[0,i] = i
					face[1,i] = i+1
					face[2,i] = nvtx+i+1
							
					face[0,nvtx+i] = nvtx+i+1
					face[1,nvtx+i] = nvtx+i
					face[2,nvtx+i] = i
				else:
					face[0,i] = i
					face[1,i] = 0
					face[2,i] = nvtx
							
					face[0,nvtx+i] = nvtx
					face[1,nvtx+i] = nvtx+i
					face[2,nvtx+i] = i

			# transformation from origin to mp
			vertex = np.vstack((vertex, np.ones((1,vertex.shape[1]))))

			axis = np.cross(l_n,np.array([0,0,1]))		
			ang_mat = np.array([axis,l_n,np.array([0,0,1])])

			if np.linalg.det(ang_mat) > 0:
				angle = -np.arccos(np.dot(l_n,np.array([0,0,1])))
			else:
				angle = np.arccos(np.dot(l_n,np.array([0,0,1])))
		
			quaternion = trans.quaternion_about_axis(angle,axis)
			rotation = trans.quaternion_matrix(quaternion)
	
			vertex = np.dot(rotation,vertex)
			vertex = vertex[:3,:]
			vertex = vertex + np.array([(mp[0]+x[0,0])*np.ones(vertex.shape[1]),(mp[1]+x[0,1])*np.ones(vertex.shape[1]),(mp[2]+x[0,2])*np.ones(vertex.shape[1])])

		else:
			vertex = np.zeros((3,1))
			face = np.zeros((3,1))

		normal = np.array([]) 		# no normal calculation (not need yet)
		obj = rviz_object(-1, vertex, face, normal)
		return obj
		'''	
		marker = Marker()
		marker.header.frame_id = "base_link"
		marker.ns = "generate_cylinder_ns"
		marker.id = 0
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD	
		marker.pose.position.x = 1
		marker.pose.position.y = 1
		marker.pose.position.z = 1
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		marker.scale.x = 1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.a = 1.0 
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0

		return marker	
		'''
	def draw_polygon(self, pts_list):
		dir_pt = pts_list[-1]
		nPts = len(pts_list)-1		
		
		center = np.zeros((3,1))
		for i in range(nPts):
			center[0] = center[0] + pts_list[i][0]
			center[1] = center[1] + pts_list[i][1]
			center[2] = center[2] + pts_list[i][2]
		
		center = center/nPts
		dist = np.linalg.norm(dir_pt - center)

		vertex = np.zeros((3, 2*(nPts+1)))
		face = np.zeros((3, 3*nPts))
		
		vertex[0,0] = center[0]
		vertex[1,0] = center[1]
		vertex[2,0] = center[2]

		vertex[:,nPts+1] = dir_pt
		for i in range(nPts):
			for j in range(3):
				vertex[j,i+1] = pts_list[i][j]
				vertex[j,i+nPts+2] = dir_pt[j] + (1.0 + dist)*(pts_list[i][j] - center[j])
			
		for i in range(nPts-1):
			# base plane
			face[0,i] = 0
			face[1,i] = i+2 
			face[2,i] = i+1

			# side plane 1
			face[0,i+nPts] = i+1			
			face[1,i+nPts] = i+2
			face[2,i+nPts] = nPts+1+i+1

			# side plane 2
			face[0,i+2*nPts] = nPts+1+i+2			
			face[1,i+2*nPts] = nPts+1+i+1
			face[2,i+2*nPts] = i+2

		# last faces (faces with nth point and 1st point)
		face[0,nPts-1] = 0
		face[1,nPts-1] = 1
		face[2,nPts-1] = nPts

		face[0,2*nPts-1] = nPts			
		face[1,2*nPts-1] = 1
		face[2,2*nPts-1] = 2*nPts+1

		# side plane 2
		face[0,3*nPts-1] = nPts+1+1			
		face[1,3*nPts-1] = 2*nPts+1
		face[2,3*nPts-1] = 1

		normal = np.array([]) 		# no normal calculation (not need yet)
		
		obj = rviz_object(-1, vertex, face, normal)
		return obj


	def plane_transform(self, norm):
		# return rotation matrix from X-Y plane to given plane with norm 
		# calculate rotation matrix
		axis = np.cross(norm,np.array([0,0,1]))
	
		ang_mat = np.array([axis,norm,np.array([0,0,1])])
		#print(ang_mat)
		if np.linalg.det(ang_mat) > 0:
			angle = -np.arccos(np.dot(norm,np.array([0,0,1])))
		else:
			angle = np.arccos(np.dot(norm,np.array([0,0,1])))
	
		quaternion = trans.quaternion_about_axis(angle,axis)
		rotation = trans.quaternion_matrix(quaternion)
		return rotation


	def triangular_normal(self, vec, pt1, pt2):
		# calculate vector(direction) toward points which makes triangular guide on the line vec (passes pt1)
		# the triangular guide faces toward pt2

		vec = vec/np.linalg.norm(vec)

		v1 = pt2 - pt1
		proj = vec.dot(v1)
		v_n = v1 - proj*vec
		v_n = v_n/np.linalg.norm(v_n)

		v_t = np.cross(vec, v_n)
		v_t = v_t/np.linalg.norm(v_t)		# for double check
		
		vec1 = 0.8*v_n + 0.6*v_t
		vec2 = 0.8*v_n - 0.6*v_t

		vec1 = vec1/np.linalg.norm(vec1)
		vec2 = vec2/np.linalg.norm(vec2)		

		return [vec1, vec2]


	def find_normal(self, vec, pt1, pt2):
		# calculate vector(direction) toward points which makes triangular guide on the line vec (passes pt1)
		# the triangular guide faces toward pt2

		vec = vec/np.linalg.norm(vec)

		v1 = pt2 - pt1
		proj = vec.dot(v1)
		v_n = v1 - proj*vec
		v_n = v_n/np.linalg.norm(v_n)

		return v_n	

	"""
	def make_STL(self, rviz_obj):
		filename_bin = os.path.join(self.obj_path,"bin_%d.stl" %rviz_obj.id)
		filename_ascii = os.path.join(self.obj_path,"ascii_%d.stl" %rviz_obj.id)
		vertex = np.transpose(rviz_obj.vertex)
		face = np.transpose(rviz_obj.face)
		face_permute = np.transpose([face[:,0], face[:,2], face[:,1]])
		face = np.concatenate((face, face_permute), axis=0)
		
		stl_mesh = mesh.Mesh(np.zeros(face.shape[0], dtype=mesh.Mesh.dtype))
		
		for i, f in enumerate(face):
			for j in range(3):
				stl_mesh.vectors[i][j] = vertex[f[j],:]

		# Write the mesh to file "cube.stl"
		stl_mesh.save(filename_bin, mode=stl.Mode.BINARY)
		stl_mesh.save(filename_ascii, mode=stl.Mode.ASCII)	
		print("stl saved")	


	#def draw_plane(self, obj_id, pts_list, show = True):
		

	def draw_circle(self, pts_list, nvtx = 20, show = True):
		center = pts_list[0] 
		point1 = pts_list[1]
		point2 = pts_list[2]

		r = np.linalg.norm(point1-center)
		x1 = point1-center
		x2 = point2-center
		norm = np.cross(x1, x2)
		print(norm)
		norm = norm/np.linalg.norm(norm)
		print(norm)
		# generate vertex and face on x-y plane
		vertex = np.zeros((3,nvtx+1))
		face = np.zeros((3,nvtx))
		face[0,:] = np.zeros((1,nvtx))
		for i in range(nvtx):
			vertex[0,i+1] = r*math.cos(2*math.pi/nvtx*i)
			vertex[1,i+1] = r*math.sin(2*math.pi/nvtx*i)
			vertex[2,i+1] = 0
			if i == nvtx-1:			# faces (center, i, i+1)
				face[1,i] = nvtx
				face[2,i] = 1
				
			else:					# last face (center, nvtx, 1)
				face[1,i] = i+1
				face[2,i] = i+2	

		# add a row of 1s for rotation matrix multiply				
		vertex = np.vstack((vertex, np.ones((1,nvtx+1))))
		#print(vertex)
		# calculate rotation matrix
		axis = np.cross(norm,np.array([0,0,1]))
		
		ang_mat = np.array([axis,norm,np.array([0,0,1])])
		print(ang_mat)
		if np.linalg.det(ang_mat) > 0:
			angle = -np.arccos(np.dot(norm,np.array([0,0,1])))
		else:
			angle = np.acos(np.dot(norm,np.array([0,0,1])))
		
		quaternion = trans.quaternion_about_axis(angle,axis)
		rotation = trans.quaternion_matrix(quaternion)
	
		# transform
		vertex = np.dot(rotation,vertex)
		vertex = vertex[:3,:]
		vertex = vertex + np.array([center[0]*np.ones(vertex.shape[1]),center[1]*np.ones(vertex.shape[1]),center[2]*np.ones(vertex.shape[1])])
		
		# generate normals of each face
		normal = np.tile(norm,(1,nvtx))
	
		# generate object
		circle = rviz_object(self.object_id, vertex, face, normal)
		self.object_id = self.object_id+1

		# make STL file
		self.make_STL(circle)

		# enlist object
		self.object_list.append(circle)
		if show: 
			self.markerArray.markers.append(circle.marker)
		
		# update haptic
		self.update = True
	

	def draw_cone(self, pts_list, nvtx = 20, show = True):
		center = pts_list[0] 
		point1 = pts_list[1]
		point2 = pts_list[2]

		r = np.linalg.norm(point1-center)
		x1 = point1-center
		x2 = point2-center
		norm = np.cross(x1, x2)
		print(norm)
		norm = norm/np.linalg.norm(norm)
		print(norm)

		# generate vertice and faces of bottom circle
		vertex1 = np.zeros((3,nvtx+1))
		face1 = np.zeros((3,nvtx))
		face1[0,:] = np.zeros((1,nvtx))
		for i in range(nvtx):
			vertex1[0,i+1] = r*math.cos(2*math.pi/nvtx*i)
			vertex1[1,i+1] = r*math.sin(2*math.pi/nvtx*i)
			vertex1[2,i+1] = 0
			if i == nvtx-1:			# faces (center, i, i+1)
				face1[1,i] = nvtx
				face1[2,i] = 1
				
			else:					# last face (center, nvtx, 1)
				face1[1,i] = i+1
				face1[2,i] = i+2	

		# generate vertice and faces of side wall
		vertex2 = np.zeros((3,nvtx+1))
		face2 = np.zeros((3,nvtx*2))
		for i in range(nvtx):
			vertex2[0,i+1] = r*1.5*math.cos(2*math.pi/nvtx*i)
			vertex2[1,i+1] = r*1.5*math.sin(2*math.pi/nvtx*i)
			vertex2[2,i+1] = 0.1
			if i == nvtx-1:			# faces (center, i, i+1)
				face2[0,2*i] = i+1
				face2[1,2*i] = 1
				face2[2,2*i] = (nvtx+1)+i+1

				face2[0,2*i+1] = 1
				face2[1,2*i+1] = (nvtx+1)+1
				face2[2,2*i+1] = (nvtx+1)+i+1
				
			else:					# last face (center, nvtx, 1)
				face2[0,2*i] = i+1
				face2[1,2*i] = i+2
				face2[2,2*i] = (nvtx+1)+i+1

				face2[0,2*i+1] = i+2
				face2[1,2*i+1] = (nvtx+1)+i+2
				face2[2,2*i+1] = (nvtx+1)+i+1	

		vertex = np.hstack((vertex1,vertex2))
		face = np.hstack((face1,face2))

		# add a row of 1s for rotation matrix multiply				
		vertex = np.vstack((vertex, np.ones((1,vertex.shape[1]))))
		#print(vertex)
		# calculate rotation matrix
		axis = np.cross(norm,np.array([0,0,1]))
		
		ang_mat = np.array([axis,norm,np.array([0,0,1])])
		print(ang_mat)
		if np.linalg.det(ang_mat) > 0:
			angle = -np.arccos(np.dot(norm,np.array([0,0,1])))
		else:
			angle = np.acos(np.dot(norm,np.array([0,0,1])))
		
		quaternion = trans.quaternion_about_axis(angle,axis)
		rotation = trans.quaternion_matrix(quaternion)
	
		# transform
		vertex = np.dot(rotation,vertex)
		vertex = vertex[:3,:]
		vertex = vertex + np.array([center[0]*np.ones(vertex.shape[1]),center[1]*np.ones(vertex.shape[1]),center[2]*np.ones(vertex.shape[1])])
		
		# generate normals of each face
		normal = np.zeros((3,face.shape[1]))
		#normal = np.tile(norm,(1,nvtx))
	
		# generate object
		cone = rviz_object(self.object_id, vertex, face, normal)
		self.object_id = self.object_id+1

		# make STL file
		self.make_STL(cone)

		# enlist object
		self.object_list.append(cone)
		if show: 
			self.markerArray.markers.append(cone.marker)
		
		# update haptic
		self.update = True


	def draw_polygon(self, pts_list, show = True):
		point0 = pts_list[0] 
		point1 = pts_list[1]
		point2 = pts_list[2]
		
		x1 = point1-point0
		x2 = point2-point0
		norm = np.cross(x1, x2)
		norm = norm/np.linalg.norm(norm)

		# generate vertice and faces of bottom circle
		vertex = np.zeros((3,3))
		vertex[:,0] = point0;
		vertex[:,1] = point1;
		vertex[:,2] = point2;
		
		face = np.zeros((3,1))
		face[0,0] = 0;
		face[1,0] = 1;
		face[2,0] = 2;

		# generate normals of each face
		normal = np.zeros((3,face.shape[1]))
		#normal = np.tile(norm,(1,nvtx))
	
		# generate object
		polygon = rviz_object(self.object_id, vertex, face, normal)
		self.object_id = self.object_id+1

		# make STL file
		self.make_STL(polygon)

		# enlist object
		self.object_list.append(polygon)
		if show: 
			self.markerArray.markers.append(polygon.marker)
		
		# update haptic
		self.update = True
	"""

		



