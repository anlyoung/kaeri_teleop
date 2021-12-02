cmake_minimum_required(VERSION 2.8.3)
project(baxter_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  std_msgs
  sensor_msgs 
  geometry_msgs
  baxter_core_msgs
  baxter_interface
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
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  baxter_core_msgs
  baxter_interface
  DEPENDS system_lib
)

# node declaration
catkin_install_python(PROGRAMS
   script/baxter_controller_teleop
   script/baxter_controller_3Drecon
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

