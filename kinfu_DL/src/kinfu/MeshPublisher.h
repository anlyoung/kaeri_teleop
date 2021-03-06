// STL
#include <iostream>
#include <vector>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// Custom
#include "parameters.h"

class MeshPublisher
{
	public:
  MeshPublisher(ros::NodeHandle &nhandle)
  {		
		nhandle.param<std::string>(PARAM_NAME_PREFIX_TOPIC,m_prefix_name,PARAM_DEFAULT_PREFIX_TOPIC);
		m_depth_optical_frame = m_prefix_name + "_depth_optical_frame";
    nhandle.param<std::string>(PARAM_NAME_MESH_TOPIC,m_mesh_topic_name,PARAM_DEFAULT_MESH_TOPIC);
    nhandle.param<std::string>(PARAM_NAME_TF_CURRENT_FRAME,m_current_frame_name,PARAM_DEFAULT_TF_CURRENT_FRAME);

		m_mesh_pub = nhandle.advertise<visualization_msgs::Marker>(m_mesh_topic_name, 1);

		m_meshes = visualization_msgs::Marker();
		m_meshes.header.frame_id = m_depth_optical_frame;
		m_meshes.ns = "RGBD_mesh";
		m_meshes.id = 0;
		m_meshes.type = visualization_msgs::Marker().TRIANGLE_LIST;
		m_meshes.action = visualization_msgs::Marker().ADD;
		m_meshes.scale.x = 1;
		m_meshes.scale.y = 1;
		m_meshes.scale.z = 1;
		m_meshes.color.r = 1;
		m_meshes.color.g = 1;
		m_meshes.color.b = 1;
		m_meshes.color.a = 1;
		m_meshes.pose = geometry_msgs::Pose();
		m_meshes.pose.orientation.w = 1;
   }

  ~MeshPublisher(){}

	void publishMeshes(std::vector<Eigen::Vector3d> & triangles, Eigen::Vector3d box_center, bool silent)
	{
		m_meshes.points.clear();
		 	for(int i = 0; i < triangles.size(); i++){
				geometry_msgs::Point p;
				p.x = triangles[i].x()+box_center.x()-m_initial_transformation.translation().x();
				p.y = triangles[i].y()+box_center.y()-m_initial_transformation.translation().y();
	 			p.z = triangles[i].z()+box_center.z()-m_initial_transformation.translation().z();
				//std::cout << p.x << "\t" << p.y << "\t" << p.z << std::endl;
				m_meshes.points.push_back(p);
			}
		m_mesh_pub.publish(m_meshes);
		if (!silent) std::cout << m_meshes.points.size() << " points published" << std::endl;
	}
	
	void setInitialTransformation(Eigen::Affine3f t){
		m_initial_transformation = t;
	}

  private:
	ros::Publisher m_mesh_pub;
	visualization_msgs::Marker m_meshes;

	std::string m_prefix_name;
	std::string m_depth_optical_frame;
	std::string m_mesh_topic_name;
	std::string m_current_frame_name;

	Eigen::Affine3f m_initial_transformation;
};
