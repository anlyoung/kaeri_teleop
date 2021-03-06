// STL
#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>

// Custom
#include "parameters.h"

using pcl::gpu::kinfuLS::KinfuTracker;

struct ImagePublisher
{
  ImagePublisher(ros::NodeHandle & nhandle)
  {
    image_transport::ImageTransport it(nhandle);
    nhandle.param<std::string>(PARAM_NAME_CURRENT_VIEW_TOPIC,m_current_view_topic,PARAM_DEFAULT_CURRENT_VIEW_TOPIC);
    m_view_publisher = it.advertise(m_current_view_topic, 10);
  }

  void
  publishScene (KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth)
  {
    kinfu.getImage (view_device_);

    int cols;
    view_device_.download (view_host_, cols);

    //convert image to sensor message
    m_msg = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    sensor_msgs::fillImage((*m_msg), "rgb8", view_device_.rows(), view_device_.cols(),
            view_device_.cols() * 3, &view_host_[0]);

    m_msg->header.frame_id=depth->header.frame_id;
    m_view_publisher.publish(m_msg);
  }

  private:
  bool paint_image_;
  bool accumulate_views_;

  KinfuTracker::View view_device_;
  KinfuTracker::View colors_device_;
  std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;

  std::string m_current_view_topic;

  //KinfuTracker::DepthMap generated_depth_;
  image_transport::Publisher m_view_publisher;

  sensor_msgs::ImagePtr m_msg;

  // forbid this constructor
  ImagePublisher() {}
};
