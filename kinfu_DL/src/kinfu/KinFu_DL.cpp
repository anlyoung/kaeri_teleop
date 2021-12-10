/*
* Software License Agreement (BSD License)
*
* Point Cloud Library (PCL) - www.pointclouds.org
* Copyright (c) 2011, Willow Garage, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/
/*
* Based on the KinfuLS ROS wrapper by Michael Korn <michael.korn(at)uni-due.de>
* http://fsstud.is.uni-due.de/svn/ros/is/kinfu/
*/
/*
* Modified by Riccardo Monica 
*   RIMLab, Department of Information Engineering, University of Parma, Italy
*   http://www.rimlab.ce.unipr.it/
* 2013-2015
*/
/*
* Modified by Donghyeon Lee
*		RNBLab, Department of Mechanical Engineering, Pohang University of Science and Technology (POSTECH), Republic of Korea 
* 2017-2018
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Linux
#include <unistd.h>

// STL
#include <iostream>
#include <vector>
#include <list>
#include <deque>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// PCL
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/angles.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

// ROS
#include <ros/ros.h>
#include <ros/spinner.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Empty.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

// Custom
#include "parameters.h"
#include "KinFuDLApp.h"

// ROS custom messages
#include <kinfu_msgs/KinfuTsdfRequest.h>

typedef pcl::ScopeTime ScopeTimeT;

using pcl::gpu::kinfuLS::KinfuTracker;
using pcl::gpu::PtrStepSz;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int print_cli_help()
{
  cout << "\nKinFu parameters:" << endl;
  cout << "    --help, -h                        : print this message" << endl;
  cout << "\nkinfuLS node parameters:" << endl;
  cout << "    volume_size <in_meters>, vs       : define integration volume size" << endl;
  cout << "    shifting_distance <in_meters>, sd : define shifting threshold (distance target-point / cube center)"
      << endl;
  cout << "    snapshot_rate <X_frames>, sr      : Extract RGB textures every <X_frames>. Default: 45" << endl;
  cout << "    extract_textures, et              : extract RGB PNG images to KinFuSnapshots folder. Default: true"
      << endl;

  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	// check arguments
  if (pcl::console::find_switch(argc, argv, "--help") ||
    pcl::console::find_switch(argc, argv, "-h"))
    return print_cli_help();

  ros::init(argc, argv, "kinfuLS");
  ros::NodeHandle nh("~");
	//ros::NodeHandle nh;

  // assign value from parameter server, with default.
  int device;
  nh.param<int>(PARAM_NAME_CUDA_DEVICE_ID, device, PARAM_DEFAULT_CUDA_DEVICE_ID);
  pcl::gpu::setDevice(device);
  pcl::gpu::printShortCudaDeviceInfo(device);

  double volume_size = PARAM_DEFAULT_VOLUME_SIZE; //pcl::device::VOLUME_SIZE
  nh.getParam(PARAM_NAME_VOLUME_SIZE, volume_size);
  nh.getParam(PARAM_SNAME_VOLUME_SIZE, volume_size);

  double shift_distance = PARAM_DEFAULT_SHIFT_DISTANCE; //pcl::device::DISTANCE_THRESHOLD;
  nh.getParam(PARAM_NAME_SHIFT_DISTANCE, shift_distance);
  nh.getParam(PARAM_SNAME_SHIFT_DISTANCE, shift_distance);

  double depth_height = PARAM_DEFAULT_DEPTH_HEIGHT, depth_width = PARAM_DEFAULT_DEPTH_WIDTH;
  nh.getParam(PARAM_NAME_DEPTH_HEIGHT,depth_height);
  nh.getParam(PARAM_NAME_DEPTH_WIDTH,depth_width);

  KinFuDLApp app(volume_size, shift_distance, nh, depth_height, depth_width);

  // start app main thread
  app.start();

  ros::spin();

  app.prepareTermination();
  app.join();

  return 0;
}
