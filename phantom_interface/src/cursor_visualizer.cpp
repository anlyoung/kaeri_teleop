// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker cs_marker;
std_msgs::String teleop_mode;
geometry_msgs::Pose cs_pose_world;

visualization_msgs::Marker updateCursor(){
	if (teleop_mode.data == "config"){
		cs_marker.type = visualization_msgs::Marker::SPHERE;
		cs_marker.action = visualization_msgs::Marker::ADD;
		cs_marker.scale.x = 0.03;    	
		cs_marker.scale.y = 0.03;			
		cs_marker.scale.z = 0.03;      		
		cs_marker.color.a = 1.0;
	}

	if (teleop_mode.data == "teleop"){
		//std::cout << "arrow" << std::endl;
	
		cs_marker.type = visualization_msgs::Marker::ARROW;
		cs_marker.action = visualization_msgs::Marker::ADD;
		cs_marker.scale.x = 0.01;	      	// shaft diameter
		cs_marker.scale.y = 0.03;					// head diameter
		cs_marker.scale.z = 0.1;      		// head length
		cs_marker.color.a = 1.0;
		cs_marker.points.clear();
		geometry_msgs::Point pt_start;
		pt_start.x = -0.2;
		pt_start.y = 0.0;
		pt_start.z = 0.0;
		geometry_msgs::Point pt_end;
		pt_end.x = 0.0;
		pt_end.y = 0.0;
		pt_end.z = 0.0;
		cs_marker.points.push_back(pt_start);
		cs_marker.points.push_back(pt_end);
	
		/*
		cs_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
		cs_marker.action = visualization_msgs::Marker::ADD;
		cs_marker.scale.x = 1.0;
		cs_marker.scale.y = 1.0;
		cs_marker.scale.z = 1.0;
		cs_marker.color.a = 1.0;
		geometry_msgs::Point pt;
		pt.x = 0.1;
		pt.y = 0.0;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = 0.0;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = -0.01;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = 0.2;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = 0.0;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.01;
		pt.y = 0.0;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = 0.0;
		pt.z = 0.3;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = 0.0;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);
		pt.x = 0.0;
		pt.y = -0.01;
		pt.z = 0.0;
		cs_marker.points.push_back(pt);		
		*/	
	}

	cs_marker.pose = cs_pose_world;
	return cs_marker;
}


void colorCursor(bool cursor_touched){
	std_msgs::String color;
	if(cursor_touched){
		color.data = "Green";
	}
	else{
		color.data = "Yellow";
	}

	if (color.data == "Yellow"){
		cs_marker.color.r = 1;
		cs_marker.color.g = 1;
		cs_marker.color.b = 0;
	}
	else if (color.data == "Green"){
		cs_marker.color.r = 0;
		cs_marker.color.g = 1;
		cs_marker.color.b = 0;
	}
	else if (color.data == "Red"){
		cs_marker.color.r = 1;
		cs_marker.color.g = 0;
		cs_marker.color.b = 0;
	}
}


void teleop_mode_callback(const std_msgs::String::ConstPtr& msg) {
	teleop_mode.data = msg->data;
}


void HIP_pose_callback(const geometry_msgs::Pose::ConstPtr& msg) {
	cs_pose_world.position.x = msg->position.x;
	cs_pose_world.position.y = msg->position.y;
	cs_pose_world.position.z = msg->position.z;

	cs_pose_world.orientation.x = msg->orientation.x;
	cs_pose_world.orientation.y = msg->orientation.y;
	cs_pose_world.orientation.z = msg->orientation.z;
	cs_pose_world.orientation.w = msg->orientation.w;
}
		

void touch_callback(const std_msgs::Bool::ConstPtr& msg) {
	colorCursor(msg->data);
}	


int main(int argc, char** argv) {
  ros::init(argc,  argv, "cursor_visualizer");
	ros::NodeHandle nh;
	
	ros::Rate loop_rate(20);
  ros::Publisher cursor_pub = nh.advertise<visualization_msgs::Marker>("cursor_mesh", 1);
	ros::Subscriber teleop_mode_sub = nh.subscribe("teleop_mode", 1, teleop_mode_callback);
  ros::Subscriber phtm_HIP_sub = nh.subscribe("Phantom_joint_states", 1, HIP_pose_callback);
	ros::Subscriber touch_sub = nh.subscribe("Phantom_touch_state", 1, touch_callback);

	// initialize variables
	cs_marker = visualization_msgs::Marker(); 
	cs_marker.header.frame_id = "world";
	cs_marker.ns = "cursor";
	cs_marker.id = 0;

	cs_pose_world.orientation.w = 1;
	teleop_mode.data = "config";


  while(ros::ok()){
		ros::spinOnce();

		// publish cursor mesh and joint
		cursor_pub.publish(updateCursor());

    loop_rate.sleep();
  }
  return 0;
}
