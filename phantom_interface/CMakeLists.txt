cmake_minimum_required(VERSION 2.8.3)
project(phantom_interface)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs visualization_msgs genmsg rosconsole tf urdf cmake_modules
)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Eigen REQUIRED)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  LIBRARIES phantom
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs genmsg rosconsole tf urdf
  DEPENDS system_lib Eigen boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(mesh_pub src/mesh_pub.cpp)
target_link_libraries(mesh_pub
  boost_system boost_signals
	HD 
	HDU 
	rt 
	ncurses 
	${catkin_LIBRARIES} 
	${OPENGL_LIBRARIES}
)

add_executable(HapticTest src/HapticFeedbackTest.cpp)
target_link_libraries(HapticTest
  boost_system boost_signals
	HD 
	HDU 
	rt 
	ncurses 
	${catkin_LIBRARIES} 
	${OPENGL_LIBRARIES}
)

add_executable(cursor_visualizer src/cursor_visualizer.cpp)
target_link_libraries(cursor_visualizer
	${catkin_LIBRARIES} 
)

add_executable(phantom 
  src/main.cpp
  src/PhantomROS.h
)

target_link_libraries(phantom 
  boost_system boost_signals
	HD 
	HDU 
	HL 
	HLU 
	rt 
	ncurses 
	${catkin_LIBRARIES} 
	${OPENGL_LIBRARIES} 
	${GLUT_LIBRARIES} 
)

#add_executable(phantom_hd 
#  src/main_hd.cpp
#  src/PhantomROS_hd.h
#)

#target_link_libraries(phantom_hd HD HDU rt ${catkin_LIBRARIES})


