// STL
#include <iostream>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

class PhantomSubscriber
{
  public:
  PhantomSubscriber(ros::NodeHandle &nhandle,tf::TransformListener & tlistener):m_tf_listener(tlistener){
		nhandle.param<std::string>(PARAM_NAME_PHANTOM_JOINT_TOPIC,m_phantom_joint_topic_name,PARAM_DEFAULT_PHANTOM_JOINT_TOPIC);
		nhandle.param<std::string>(PARAM_NAME_PREFIX_TOPIC,m_prefix_name,PARAM_DEFAULT_PREFIX_TOPIC);
		m_depth_optical_frame = m_prefix_name + "_depth_optical_frame";

		m_joint_sub = nhandle.subscribe(m_phantom_joint_topic_name, 10, &PhantomSubscriber::jointCallback, this);

		m_joint_pose_phtm = geometry_msgs::Pose();
		m_joint_pose_phtm.orientation.w = 1.0;
		m_joint_pose_tsdf = geometry_msgs::Pose();
		m_joint_pose_tsdf.orientation.w = 1.0;

   }

  ~PhantomSubscriber(){}

  void jointCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
		//printf("joint updated (%f %f %f) \n", msg->position.x, msg->position.y, msg->position.z);
		m_joint_pose_phtm = geometry_msgs::Pose();
    m_joint_pose_phtm.position.x = msg->position.x;
		m_joint_pose_phtm.position.y = msg->position.y;
		m_joint_pose_phtm.position.z = msg->position.z;
		m_joint_pose_phtm.orientation.w = 1.0;
  }

  void getCursor(tf::Vector3 & cursor_tsdf)
  {
		ros::Time now = ros::Time::now();
    if(m_tf_listener.waitForTransform(m_depth_optical_frame, "world", now,ros::Duration(0.1))){
		  tf::StampedTransform tf_phtm2kinect;		
			try{
			    m_tf_listener.lookupTransform(m_depth_optical_frame, "world", now, tf_phtm2kinect);
			}
			catch(tf::TransformException ex){
			    ROS_ERROR("%s", ex.what());
			}
			tf::Vector3 cursor_phtm = tf::Vector3(m_joint_pose_phtm.position.x, m_joint_pose_phtm.position.y, m_joint_pose_phtm.position.z);
			tf::Vector3 cursor_kinect = tf_phtm2kinect(cursor_phtm);	
			Eigen::Vector3f trans = m_initial_transformation.translation();
			tf::Transform tf_kinect2tsdf(tf::Quaternion(0,0,0,1), tf::Vector3(trans.x(), trans.y(), trans.z()));
			cursor_tsdf = tf_kinect2tsdf(cursor_kinect);
			m_joint_pose_tsdf.position.x = cursor_tsdf.x();
			m_joint_pose_tsdf.position.y = cursor_tsdf.y();
			m_joint_pose_tsdf.position.z = cursor_tsdf.z();
			//printf("(%f, %f, %f)\n", cursor_kinect.x(), cursor_kinect.y(), cursor_kinect.z());		
		}
		else{
			std::cout<<"[PhantomSubscriber] cannot find frame (" << m_depth_optical_frame << " to world" << ")" << std::endl;
			cursor_tsdf = tf::Vector3(m_joint_pose_tsdf.position.x, m_joint_pose_tsdf.position.y, m_joint_pose_tsdf.position.z);
		}		
  }

	void getBoxCenter(Eigen::Vector3d & box_center){
		box_center.x() = m_joint_pose_tsdf.position.x - m_initial_transformation.translation().x();
		box_center.y() = m_joint_pose_tsdf.position.y - m_initial_transformation.translation().y();
		box_center.z() = m_joint_pose_tsdf.position.z - m_initial_transformation.translation().z();	
	}

	void setInitialTransformation(Eigen::Affine3f t){
		m_initial_transformation = t;
	}

  private:
	std::string m_phantom_joint_topic_name;
	std::string m_prefix_name;
	std::string m_depth_optical_frame;

	ros::Subscriber m_joint_sub;

	tf::TransformListener & m_tf_listener;

	Eigen::Affine3f m_initial_transformation;

	geometry_msgs::Pose m_joint_pose_phtm;
	geometry_msgs::Pose m_joint_pose_tsdf;  
};

