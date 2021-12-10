// STL
#include <iostream>
#include <vector>

// Eigen
#include <Eigen/Core>

// ROS
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

class RobotSubscriber
{
  public:
  RobotSubscriber(ros::NodeHandle &nhandle,tf::TransformListener & tlistener): m_tf_listener(tlistener){
		nhandle.param<std::string>(PARAM_NAME_PREFIX_TOPIC,m_prefix_name,PARAM_DEFAULT_PREFIX_TOPIC);
		m_depth_optical_frame = m_prefix_name + "_depth_optical_frame";
		m_camera_link_frame = m_prefix_name + "_link";

    nhandle.param<std::string>(PARAM_NAME_TF_REFERENCE_FRAME,m_first_frame_name,PARAM_DEFAULT_TF_REFERENCE_FRAME);
  }


  ~RobotSubscriber(){}


  bool getRobotJoints(std::vector<Eigen::Vector3f> & robot_joints, geometry_msgs::Pose& RGBD_sensor_pose)
  {
		bool robot_info = false;
		//ros::Time now = ros::Time(0);
		ros::Time now = ros::Time::now();
		robot_joints.clear();
		
		// right_lower_elbow
	  tf::StampedTransform tf_r_l_e;			// tf_right_lower_elbow		
		try{
		    m_tf_listener.lookupTransform(m_depth_optical_frame, "right_lower_elbow", now, tf_r_l_e);
				robot_joints.push_back(Eigen::Vector3f(tf_r_l_e.getOrigin().x(), 
																							tf_r_l_e.getOrigin().y(), 
																							tf_r_l_e.getOrigin().z()));
		}
		catch(tf::TransformException ex){
		    ROS_ERROR("%s", ex.what());
				std::cout<<"[RobotSubscriber] cannot find frame (" << m_depth_optical_frame << " to right_lower_elbow" << ")" << std::endl;
				robot_joints.push_back(Eigen::Vector3f(0,0,0));
				return robot_info;
		}
					
		// right_lower_forearm
	  tf::StampedTransform tf_r_l_f;			// tf_right_lower_forearm		
		try{
	    m_tf_listener.lookupTransform(m_depth_optical_frame, "right_lower_forearm", now, tf_r_l_f);
			robot_joints.push_back(Eigen::Vector3f(tf_r_l_f.getOrigin().x(), 
																						tf_r_l_f.getOrigin().y(), 
																						tf_r_l_f.getOrigin().z()));
		}
		catch(tf::TransformException ex){
	    ROS_ERROR("%s", ex.what());
			std::cout<<"[RobotSubscriber] cannot find frame (" << m_depth_optical_frame << " to right_lower_forearm" << ")" << std::endl;
			robot_joints.push_back(Eigen::Vector3f(0,0,0));
			return robot_info;
		}
				
		// right_hand
	  tf::StampedTransform tf_r_h;				// tf_right_hand	
		try{
	    m_tf_listener.lookupTransform(m_depth_optical_frame, "right_hand", now, tf_r_h);
			robot_joints.push_back(Eigen::Vector3f(tf_r_h.getOrigin().x(), 
																						tf_r_h.getOrigin().y(), 
																						tf_r_h.getOrigin().z()));
		}
		catch(tf::TransformException ex){
	    ROS_ERROR("%s", ex.what());
			std::cout<<"[RobotSubscriber] cannot find frame (" << m_depth_optical_frame << " to right_hand)" << std::endl;
			robot_joints.push_back(Eigen::Vector3f(0,0,0));
			return robot_info;
		}
		
		// left_gripper (RGBD sensor)
	  tf::StampedTransform sensor_pose;		// tf_hint	
		try{
	    m_tf_listener.lookupTransform("world", m_camera_link_frame, now, sensor_pose);
			RGBD_sensor_pose.position.x = sensor_pose.getOrigin().x();
			RGBD_sensor_pose.position.y = sensor_pose.getOrigin().y();
			RGBD_sensor_pose.position.z = sensor_pose.getOrigin().z();

			RGBD_sensor_pose.orientation.x = sensor_pose.getRotation().x();
			RGBD_sensor_pose.orientation.y = sensor_pose.getRotation().y();
			RGBD_sensor_pose.orientation.z = sensor_pose.getRotation().z();
			RGBD_sensor_pose.orientation.w = sensor_pose.getRotation().w();
		}
		catch(tf::TransformException ex){
	    ROS_ERROR("%s", ex.what());
			std::cout<<"[RobotSubscriber] cannot find frame (world to " << m_camera_link_frame << ")" << std::endl;
		}
			
		return true;		
  }

  private:
  std::string m_prefix_name;
  std::string m_depth_optical_frame;
	std::string m_camera_link_frame;
  std::string m_first_frame_name;	



	tf::TransformListener & m_tf_listener;
};
