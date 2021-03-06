// STL
#include <iostream>
#include <fstream>
#include <string>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>

// custom
//#include "../kinfu/parameters.h"

#define PI							3.14159265359			
#define TRANS_STEP			0.01
#define ROT_STEP 				PI/180.0


geometry_msgs::Pose RGBD_link_pose;

//used to convert data from kinect_pos_config.txt
double stof2(std::string value){
  const char *s = value.c_str();
  double rez = 0, fact = 1;
  if (*s == '-'){
    s++;
    fact = -1;
  };
  for (int point_seen = 0; *s; s++){
    if (*s == '.'){
      point_seen = 1; 
      continue;
    };
    int d = *s - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0;
      rez = rez * 10.0 + (double)d;
    };
  };
  return rez * fact;
};

void translate_link(int axis, bool sign){
	float movement;
	if(sign) movement = TRANS_STEP;	
	else movement = -TRANS_STEP;

	if(axis==0) RGBD_link_pose.position.x += movement;
	else if(axis==1) RGBD_link_pose.position.y += movement;
	else if(axis==2) RGBD_link_pose.position.z += movement;
	std::cout << RGBD_link_pose.position.x << "," << RGBD_link_pose.position.y << "," << RGBD_link_pose.position.z << std::endl;
}


void rotate_link(int axis, bool sign){
	float movement;
	if(sign) movement = ROT_STEP;	
	else movement = -ROT_STEP;

	tf::Vector3 rot_axis;
	if(axis==0) rot_axis = tf::Vector3(1,0,0);
	else if(axis==1) rot_axis = tf::Vector3(0,1,0);
	else if(axis==2) rot_axis = tf::Vector3(0,0,1);

	tf::Transform rot;
	rot.setRotation(tf::Quaternion(rot_axis, movement));
	tf::Quaternion ori = tf::Quaternion(RGBD_link_pose.orientation.x, RGBD_link_pose.orientation.y, RGBD_link_pose.orientation.z, RGBD_link_pose.orientation.w);
	ori = rot*ori;
	
	RGBD_link_pose.orientation.x = ori.x();
	RGBD_link_pose.orientation.y = ori.y();
	RGBD_link_pose.orientation.z = ori.z();
	RGBD_link_pose.orientation.w = ori.w();	
}


void move_link_callback(const std_msgs::String::ConstPtr& msg) {
	if (msg->data == "pos_x+") translate_link(0,true);
	if (msg->data == "pos_x-") translate_link(0,false);
	if (msg->data == "pos_y+") translate_link(1,true);
	if (msg->data == "pos_y-") translate_link(1,false);
	if (msg->data == "pos_z+") translate_link(2,true);
	if (msg->data == "pos_z-") translate_link(2,false);

	if (msg->data == "ori_x+") rotate_link(0,true);
	if (msg->data == "ori_x-") rotate_link(0,false);
	if (msg->data == "ori_y+") rotate_link(1,true);
	if (msg->data == "ori_y-") rotate_link(1,false);
	if (msg->data == "ori_z+") rotate_link(2,true);
	if (msg->data == "ori_z-") rotate_link(2,false);
	std::cout << msg->data << std::endl;
}	


int main(int argc, char** argv) {

  ros::init(argc,  argv, "RGBD_link_tf_broadcaster");
	ros::NodeHandle nh("~");

	std::string node_name;
	std::string prefix_name;
	std::string RGBD_link_frame;
	node_name = nh.getNamespace();
	nh.getParam(node_name+"/prefix_topic",prefix_name);
	RGBD_link_frame = prefix_name + "_link";

	std::cout << "Prefix Name" << prefix_name << std::endl;

	tf::TransformBroadcaster tf_broadcaster;	


	//load default from a kinect_pos_config.txt
	std::string path = ros::package::getPath("kinfu");  //get path file for kinfu
	std::string location = "/src/RGBD_link_tf_broadcaster/kinect_pos_config.txt";
	std::string full = path + location;
	std::ifstream file ( full.c_str()); 
	std::cout << "Location of kinect_position: " << full << std::endl;
  //std::ifstream file ( "/home/anl-testbed/catkin_ws_working_copy/src/kinfu_DL/src/RGBD_link_tf_broadcaster/kinect_pos_config.txt"); //full path file
  std::string value;

	RGBD_link_pose = geometry_msgs::Pose();
	getline ( file, value, ',' );
	RGBD_link_pose.position.x =  stof2(value);

	getline ( file, value, ',' );
	RGBD_link_pose.position.y =  stof2(value);

	getline ( file, value, ',' );
	RGBD_link_pose.position.z =   stof2(value);

	getline ( file, value, ',' );
  RGBD_link_pose.orientation.x =   stof2(value);	// Pose orientation values will

	getline ( file, value, ',' );
	RGBD_link_pose.orientation.y =stof2(value);// shift the whole environment 

	getline ( file, value, ',' );
	RGBD_link_pose.orientation.z =  stof2(value);// in RViz, especailly w.

	getline ( file, value, ',' );
  RGBD_link_pose.orientation.w = stof2(value);

	file.close();
	std::cout << "Kinect position taken from: " << full << std::endl;

	/*
	//manually enter values below if needed
	RGBD_link_pose = geometry_msgs::Pose();
	RGBD_link_pose.position.x = 0.10094308 ;
	RGBD_link_pose.position.y = -0.00568244  ;
	RGBD_link_pose.position.z =  0.948598  ;
  RGBD_link_pose.orientation.x =  0.00792832   ;	// Pose orientation values will
	RGBD_link_pose.orientation.y =  0.32304914   ;		// shift the whole environment 
	RGBD_link_pose.orientation.z =    0.01671893;	// in RViz, especailly w.
  RGBD_link_pose.orientation.w =  0.94620129 ;*/

	/*RGBD_link_pose = geometry_msgs::Pose();
	RGBD_link_pose.position.x = 0.09765124 ;
	RGBD_link_pose.position.y = 0.00362746 ;
	RGBD_link_pose.position.z =  0.94718775;
  RGBD_link_pose.orientation.x =  0.00531248;	// Pose orientation values will
	RGBD_link_pose.orientation.y =  0.3177973 ;		// shift the whole environment 
	RGBD_link_pose.orientation.z =  0.02606177;	// in RViz, especailly w.
  RGBD_link_pose.orientation.w =  0.94778554;
	*/
// 0, -0.0399, -0.03
//	RGBD_link_pose.position.x = 0.28691723;
//	RGBD_link_pose.position.y = 0.39320861;
//	RGBD_link_pose.position.z = 1.02000998;
 // RGBD_link_pose.orientation.x =  0.01447967;	// Pose orientation values will
	//RGBD_link_pose.orientation.y = 0.26098965;		// shift the whole environment 
	//RGBD_link_pose.orientation.z = -0.2856302;	// in RViz, especailly w.
  //RGBD_link_pose.orientation.w = 0.92200333;

	ros::Rate loop_rate(50);

	ros::Subscriber move_link_sub = nh.subscribe("/RGBD_link_move", 1, move_link_callback);

  while(ros::ok()){
		ros::spinOnce();

		// broadcast RGBD_link frame
		tf::Transform RGBD_link;
		tf::Vector3 origin(RGBD_link_pose.position.x,RGBD_link_pose.position.y,RGBD_link_pose.position.z);
		tf::Quaternion rotation(RGBD_link_pose.orientation.x,RGBD_link_pose.orientation.y,RGBD_link_pose.orientation.z,RGBD_link_pose.orientation.w);
		RGBD_link.setOrigin(origin);
		RGBD_link.setRotation(rotation);
		//ROS_INFO("[RGBD_sensor] pos: (%f, %f, %f)", RGBD_link_pose.position.x, RGBD_link_pose.position.y, RGBD_link_pose.position.z);
		//ROS_INFO("[RGBD_sensor] ori: (%f, %f, %f, %f)", RGBD_link_pose.orientation.x, RGBD_link_pose.orientation.y, RGBD_link_pose.orientation.z, RGBD_link_pose.orientation.w);
    ros::Duration sleeper(-2.0);
    //ros::Duration sleeper_with_eps(0.12);
    ros::WallTime time_stamp = ros::WallTime::now();
		//tf::StampedTransform world2link = tf::StampedTransform(RGBD_link, ros::Time::now(), "/world", RGBD_link_frame);
    tf::StampedTransform world2link = tf::StampedTransform(RGBD_link,ros::Time::now(), "/world", RGBD_link_frame);
    //std::cout << "TIME: " << ros::Time::now() << std::endl;
    //std::cout << "Wall_TIME: " << time_stamp << std::endl;
		tf_broadcaster.sendTransform(world2link);
    loop_rate.sleep();
  }
  return 0;
}
