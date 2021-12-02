cmake_minimum_required(VERSION 2.8.3)
project(input_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  baxter_examples
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
  baxter_examples
  DEPENDS system_lib
)

# node declaration
catkin_install_python(PROGRAMS
   script/input_manager
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

