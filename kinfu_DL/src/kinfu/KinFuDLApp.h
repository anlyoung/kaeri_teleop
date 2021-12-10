// STL
#include <iostream>
#include <vector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Eigen
#include <Eigen/Core>

// PCL
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>

// PCL/GPU
#include <pcl/gpu/kinfu_large_scale/kinfu.h>

// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Custom
#include "CommandSubscriber.h"
#include "ImagePublisher.h"
#include "ImageSubscriber.h"
#include "MarchingCubeDL.h"
#include "MeshPublisher.h"
#include "parameters.h"
#include "PhantomSubscriber.h"
#include "PosePublisher.h"
#include "RobotSubscriber.h"

typedef unsigned int uint;

using pcl::gpu::PtrStepSz;

struct SampledScopeTime : public pcl::StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    time_ms_ += getTime();
    if (i_ % EACH == 0 && i_)
    {
      ROS_INFO("Avg frame time = %.2f ms (%.2f fps)",float(time_ms_) / EACH,float(1000.f * EACH / time_ms_));
      time_ms_ = 0;
    }
    ++i_;
  }
private:
  int& time_ms_;
};


class KinFuDLApp
{
  enum
  {
    PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8
  };

public:
  KinFuDLApp(float vsz, float shiftDistance, ros::NodeHandle & nodeHandle, uint depth_height, uint depth_width) :
			m_image_subscriber(nodeHandle,m_mutex,m_cond),
			m_phantom_subscriber(nodeHandle,m_tf_listener),								
			m_robot_subscriber(nodeHandle,m_tf_listener),	
			m_command_subscriber(nodeHandle),								
			m_mesh_publisher(nodeHandle),																	
      m_image_publisher(nodeHandle), 
			m_pose_publisher(nodeHandle), 						
      m_marching_cube(),
      time_ms_(0), nh(nodeHandle)
  {
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(vsz/*meters*/);

    ROS_INFO("--- CURRENT SETTINGS ---\n");
    ROS_INFO("Volume size is set to %.2f meters\n", vsz);
    ROS_INFO("Volume will shift when the camera target point is farther than %.2f meters from the volume center\n", shiftDistance);
    ROS_INFO("The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6*vsz);
    ROS_INFO("------------------------\n");

    // warning message if shifting distance is abnormally big compared to volume size
    if(shiftDistance > 2.5 * vsz)
      ROS_WARN("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n", shiftDistance, vsz);

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shiftDistance, depth_height, depth_width);
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Eigen::Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    m_pose_publisher.setReverseInitialTransformation(pose.inverse());
		m_pose_publisher.setInitialTransformation(pose);										
		m_mesh_publisher.setInitialTransformation(pose);										
		m_phantom_subscriber.setInitialTransformation(pose);								

    kinfu_->setInitialCameraPose(pose);
    kinfu_->volume().setTsdfTruncDist(0.030f/*meters*/);
    kinfu_->setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.f)));
    //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
    kinfu_->setCameraMovementThreshold(0.001f);

    //Init KinFuLSApp
    tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    frame_counter_ = 0;
    enable_texture_extraction_ = false;

    m_request_termination = false;
  }

  ~KinFuDLApp()
  {
  }

	void run();
	void execute(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
               const sensor_msgs::ImageConstPtr& rgb,const KinfuTracker::THint & pose_hint);
  void start();
  void prepareTermination();
  void join();

private:
  int frame_counter_;
  bool enable_texture_extraction_;

  KinfuTracker *kinfu_;

  ImageSubscriber m_image_subscriber;
  PhantomSubscriber m_phantom_subscriber;				
  RobotSubscriber m_robot_subscriber;			
	CommandSubscriber m_command_subscriber;	

  ImagePublisher m_image_publisher;
  PosePublisher m_pose_publisher;	
  MeshPublisher m_mesh_publisher;				
				
	MarchingCubeDL m_marching_cube;	

  KinfuTracker::DepthMap depth_device_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

  boost::thread m_thread;
  bool m_request_termination;

  std::vector<pcl::gpu::kinfuLS::PixelRGB> source_image_data_;
  std::vector<unsigned short> source_depth_data_;
  PtrStepSz<const unsigned short> depth_;
  PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24_;

  int time_ms_;
  std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;
  KinfuTracker::DepthMap generated_depth_;

  tf::TransformListener m_tf_listener;

  //the ros node handle used to shut down the node and stop capturing
  ros::NodeHandle & nh;

  boost::mutex m_mutex;
  boost::condition_variable m_cond;
};
