cmake_minimum_required(VERSION 2.8.3)
project(vf_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf
)

# for setup.py
catkin_python_setup()

#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf
  DEPENDS system_lib
)

# node declaration
catkin_install_python(PROGRAMS
   script/vf_interface
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

