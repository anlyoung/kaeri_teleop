cmake_minimum_required(VERSION 2.8.3)
project(vr_baxter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  PCL
  std_msgs
  message_generation
#  vr_interface_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vr_baxter
  CATKIN_DEPENDS roscpp rospy
  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime #vr_interface_msgs
)

 add_message_files(
   FILES
   FloatArray.msg
 )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(EuclideanExtraction src/EuclideanExtraction.cpp)
target_link_libraries(EuclideanExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(EuclideanExtraction vr_interface_msgs_gencpp)

add_executable(basic_shapes src/MarkerPub.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})


## Practice for service (if you are using this, uncomment those lines)
#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server vrep_interface_msgs_gencpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client vrep_interface_msgs_gencpp)
