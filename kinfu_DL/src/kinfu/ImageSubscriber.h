// STL
#include <iostream>

// Boost
#include <boost/thread/mutex.hpp>

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Custom
#include "parameters.h"

using sensor_msgs::CameraInfo;
using sensor_msgs::Image;

class ImageSubscriber
{
  public:
  ImageSubscriber(ros::NodeHandle &nhandle,boost::mutex & shared_mutex,boost::condition_variable & cond): 
    m_shared_mutex(shared_mutex),m_cond(cond),m_nh(nhandle)
  {
    std::string prefix_topic;
    m_nh.param<std::string>(PARAM_NAME_PREFIX_TOPIC,prefix_topic,PARAM_DEFAULT_PREFIX_TOPIC);

    std::string depth_image_topic;
    m_nh.param<std::string>(PARAM_NAME_DEPTH_IMAGE_TOPIC,depth_image_topic,prefix_topic + PARAM_DEFAULT_DEPTH_IMAGE_TOPIC);

    std::string camera_info_topic;
    m_nh.param<std::string>(PARAM_NAME_CAMERA_INFO_TOPIC,camera_info_topic,prefix_topic + PARAM_DEFAULT_CAMERA_INFO_TOPIC);

    std::string image_topic;
    m_nh.param<std::string>(PARAM_NAME_IMAGE_TOPIC,image_topic,prefix_topic + PARAM_NAME_IMAGE_TOPIC);

    bool enable_texture_extraction = PARAM_DEFAULT_EXTRACT_TEXTURES;
    m_nh.getParam(PARAM_NAME_EXTRACT_TEXTURES,enable_texture_extraction);
    m_nh.getParam(PARAM_SNAME_EXTRACT_TEXTURES,enable_texture_extraction);

    // message_filters instead of image_transport because of synchronization over w-lan
    m_texture_sync = NULL;
    m_depth_only_sync = NULL;

    m_rgb_sub = NULL;
    m_depth_sub = NULL;
    m_info_sub = NULL;

    if (enable_texture_extraction)
    {
      m_depth_sub = new message_filters::Subscriber<Image>(m_nh, depth_image_topic, 2);
      m_info_sub  = new message_filters::Subscriber<CameraInfo>(m_nh, camera_info_topic, 2);
      m_rgb_sub   = new message_filters::Subscriber<Image>(m_nh, image_topic, 2);

      //the depth and the rgb cameras are not hardware synchronized
      //hence the depth and rgb images normally do not have the EXACT timestamp
      //so use approximate time policy for synchronization
      m_texture_sync = new message_filters::Synchronizer<DRGBSync>(DRGBSync(500), *m_depth_sub, *m_info_sub, *m_rgb_sub);
      m_texture_sync->registerCallback(boost::bind(&ImageSubscriber::imageCallback, this, _1, _2, _3));
      ROS_INFO("Running KinFu with texture extraction");
    }
    else
    {
      m_depth_sub = new message_filters::Subscriber<Image>(m_nh, depth_image_topic, 1);
      m_info_sub  = new message_filters::Subscriber<CameraInfo>(m_nh, camera_info_topic, 1);

      m_depth_only_sync = new message_filters::TimeSynchronizer<Image, CameraInfo>(*m_depth_sub, *m_info_sub, 500);
      m_depth_only_sync->registerCallback(boost::bind(&ImageSubscriber::imageCallback, this, _1, _2, sensor_msgs::ImageConstPtr()));
      ROS_INFO("Running KinFu without texture extraction");
    }

    m_has_image = false;
  }

  ~ImageSubscriber()
  {
    if (m_texture_sync)
      delete m_texture_sync;
    if (m_depth_only_sync)
      delete m_depth_only_sync;
    if (m_depth_sub)
      delete m_depth_sub;
    if (m_info_sub)
      delete m_info_sub;
    if (m_rgb_sub)
      delete m_rgb_sub;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
    const sensor_msgs::ImageConstPtr& rgb = sensor_msgs::ImageConstPtr())
  {
		//printf("image updated\n");
    boost::mutex::scoped_lock lock(m_shared_mutex);
    m_depth = depth;
    m_cameraInfo = cameraInfo;
    m_rgb = rgb;

    m_has_image = true;
    m_cond.notify_one();
  }

  // WARNING: lock the shared mutex before calling this
  bool hasImage()
  {
    return m_has_image;
  }

  // WARNING: lock the shared mutex before calling this
  void getImage(sensor_msgs::ImageConstPtr & depth,sensor_msgs::CameraInfoConstPtr & cameraInfo,sensor_msgs::ImageConstPtr & rgb)
  {
    if (!m_has_image)
      return;    

    depth = m_depth;
    cameraInfo = m_cameraInfo;
    rgb = m_rgb;
  }

  // WARNING: lock the shared mutex before calling this
  void clearImage()
  {
    m_has_image = false;
  }

  private:

  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image> DRGBSync;
  message_filters::Synchronizer<DRGBSync>* m_texture_sync;
  message_filters::TimeSynchronizer<Image, CameraInfo>* m_depth_only_sync;

  message_filters::Subscriber<Image>* m_rgb_sub;
  message_filters::Subscriber<Image>* m_depth_sub;
  message_filters::Subscriber<CameraInfo>* m_info_sub;

  sensor_msgs::ImageConstPtr m_rgb;
  sensor_msgs::ImageConstPtr m_depth;
  sensor_msgs::CameraInfoConstPtr m_cameraInfo;

  boost::mutex & m_shared_mutex;
  boost::condition_variable & m_cond;

  ros::NodeHandle & m_nh;

  bool m_has_image;
};
