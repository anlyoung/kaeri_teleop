#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <pthread.h>


void msgCallback(const sensor_msgs::JointState::ConstPtr& joint_state){
    ROS_INFO("%.3f %.3f %.3f %.3f %.3f %.3f", joint_state->position[0], joint_state->position[1], joint_state->position[2], joint_state->position[3], joint_state->position[4], joint_state->position[5]);
}

int main(int argc, char** argv) {
    ros::init(argc,  argv, "omni_test_node");
    ros::Subscriber joints_sub;
    ros::NodeHandle nh;

    joints_sub = nh.subscribe("omni1_joint_states", 1, msgCallback);

	ros::spin();
	return 0;
}
