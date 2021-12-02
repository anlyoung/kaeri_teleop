cmake_minimum_required(VERSION 2.8.3)

project(vr_interface_msgs)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  message_generation 
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg
                  FILES
		FloatArray.msg
		PoseNameArray.msg
)

add_service_files(
   FILES
   AddTwoInts.srv	# you can modify this
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
