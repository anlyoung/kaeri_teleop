cmake_minimum_required(VERSION 2.8.3)

project(sawyer_interface_msgs)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  message_generation 
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg
                  FILES
                  SimStatus.msg
		              AdcCtrlStatus.msg
)

add_service_files(
   FILES
   CheckArrived.srv
   CheckCollision.srv
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
