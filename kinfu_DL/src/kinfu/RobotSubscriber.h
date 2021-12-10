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
		ros::Time now = ros::Time(0);
		//ros::Time now = ros::Time::now();
		robot_joints.clear();
		
		// depth_optical_frame to world
		tf::StampedTransform tf_w2d;				// tf from world to depth_optical_frame
		if (m_prefix_name == "kinect"){ 
			try{
				  m_tf_listener.lookupTransform(m_depth_optical_frame, "world", now, tf_w2d);
			}
			catch(tf::TransformException ex){
				  ROS_ERROR("%s", ex.what());
					std::cout<<"[RobotSubscriber] cannot find frame (" << "world to " << m_depth_optical_frame << ")" << std::endl;
					robot_joints.push_back(Eigen::Vector3f(0,0,0));
					return robot_info;
			}
		}
		else if (m_prefix_name == "xtion"){
			tf::StampedTransform tf_lg2d;				// tf from left_gripper to depth
			tf::StampedTransform tf_w2lg;				// tf from world to left_gripper
			try{
				  m_tf_listener.lookupTransform(m_depth_optical_frame, "left_gripper", now, tf_lg2d);
				  m_tf_listener.lookupTransform("left_gripper", "world", now, tf_w2lg);
			}
			catch(tf::TransformException ex){
				  ROS_ERROR("%s", ex.what());
					std::cout<<"[RobotSubscriber] cannot find frame (" << "world to " << m_depth_optical_frame << ")" << std::endl;
					robot_joints.push_back(Eigen::Vector3f(0,0,0));
					return robot_info;
			}
			tf_w2d.setData(tf_lg2d*tf_w2lg);
		}

		// world to right_lower_elbow
	  tf::StampedTransform tf_rle2w;			// tf from right_lower_elbow to world 	
		try{
		    m_tf_listener.lookupTransform("world", "right_lower_elbow", now, tf_rle2w);
		}
		catch(tf::TransformException ex){
		    ROS_ERROR("%s", ex.what());
				std::cout<<"[RobotSubscriber] cannot find frame (" << "right_lower_elbow" << " to world" << ")" << std::endl;
				robot_joints.push_back(Eigen::Vector3f(0,0,0));
				return robot_info;
		}
					
		// world to right_lower_forearm
	  tf::StampedTransform tf_rlf2w;			// tf from right_lower_forearm to world
		try{
	    m_tf_listener.lookupTransform("world", "right_lower_forearm", now, tf_rlf2w);			
		}
		catch(tf::TransformException ex){
	    ROS_ERROR("%s", ex.what());
			std::cout<<"[RobotSubscriber] cannot find frame (" << "right_lower_forearm" << " to world" << ")" << std::endl;
			robot_joints.push_back(Eigen::Vector3f(0,0,0));
			return robot_info;
		}
				
		// world to right_hand
	  tf::StampedTransform tf_rh2w;				// tf from right_hand	to world
		try{
	    m_tf_listener.lookupTransform("world", "right_hand", now, tf_rh2w);			
		}
		catch(tf::TransformException ex){
	    ROS_ERROR("%s", ex.what());
			std::cout<<"[RobotSubscriber] cannot find frame (" << "right_hand" << " to world" << ")" << std::endl;
			robot_joints.push_back(Eigen::Vector3f(0,0,0));
			return robot_info;
		}
		tf::Transform tf_rle2d, tf_rlf2d, tf_rh2d;
		tf_rle2d = tf_w2d*tf_rle2w;
		tf_rlf2d = tf_w2d*tf_rlf2w;
		tf_rh2d = tf_w2d*tf_rh2w;

		robot_joints.push_back(Eigen::Vector3f(tf_rle2d.getOrigin().x(), 
																					tf_rle2d.getOrigin().y(), 
																					tf_rle2d.getOrigin().z()));

		robot_joints.push_back(Eigen::Vector3f(tf_rlf2d.getOrigin().x(), 
																					tf_rlf2d.getOrigin().y(), 
																					tf_rlf2d.getOrigin().z()));

		robot_joints.push_back(Eigen::Vector3f(tf_rh2d.getOrigin().x(), 
																					tf_rh2d.getOrigin().y(), 
																					tf_rh2d.getOrigin().z()));


		// world to RGBD_sensor_link
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
