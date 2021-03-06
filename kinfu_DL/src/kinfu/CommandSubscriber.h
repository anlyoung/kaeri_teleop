// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

// custom
#include "parameters.h"

class CommandSubscriber
{
  public:
  CommandSubscriber(ros::NodeHandle &nhandle){
		nhandle.param<std::string>(PARAM_NAME_COMMAND_TOPIC,m_command_topic_name,PARAM_DEFAULT_COMMAND_TOPIC);
		m_command_sub = nhandle.subscribe(m_command_topic_name, 1, &CommandSubscriber::commandCallback, this);
		env_recon = true;
  }


  ~CommandSubscriber(){}


	void commandCallback(const std_msgs::Bool::ConstPtr& msg)
	{
		env_recon = msg->data;
		if(env_recon) std::cout << "[CommandSubscriber] mesh generation enabled" << std::endl;
		else std::cout << "[CommandSubscriber] mesh generation disabled" << std::endl;
	}


  bool extract_mesh()
  {
		return env_recon;
  }


  private:
	std::string m_command_topic_name;

	ros::Subscriber m_command_sub;
	
	bool env_recon;
};
