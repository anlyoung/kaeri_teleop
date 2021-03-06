#include "KinFuDLApp.h"

// this contains the main cycle for the kinfu thread
void KinFuDLApp::run()
{
  boost::mutex::scoped_lock main_lock(m_mutex);

  while (!m_request_termination)
  {
    bool reset;
    std::string reset_command_id;
    bool hasimage;
    bool hasrequests;
    bool istriggered;

    KinfuTracker::THint pose_hint;
	
  	hasimage = m_image_subscriber.hasImage();
    while (!m_request_termination && !hasimage){
	  	hasimage = m_image_subscriber.hasImage();
      m_cond.wait(main_lock);
		}
		/*
    // command execution is allowed only when a new image is available
    if (hasimage)
    {
      if (m_command_subscriber.isResetRequired())
      {
        reset = true;
        reset_command_id = m_command_subscriber.getResetCommandId();
        m_command_subscriber.clearResetRequired();
      }

      if (m_command_subscriber.hasHint())
      {
        pose_hint.type = m_command_subscriber.hasForcedHint() ?
          KinfuTracker::THint::HINT_TYPE_FORCED : KinfuTracker::THint::HINT_TYPE_HINT;
        pose_hint.transform = m_command_subscriber.getHintTransform();
        m_command_subscriber.clearHint();
      }

      pose_hint.ignore_minimum_movement = !m_command_subscriber.isEnabledMinimumMovement();
    }
		*/
    sensor_msgs::ImageConstPtr depth;
    sensor_msgs::CameraInfoConstPtr cameraInfo;
    sensor_msgs::ImageConstPtr rgb;

    // Krebs
//		std::cout << "TEST" << std::endl;
      
    if (hasimage)
    {
      m_image_subscriber.getImage(depth,cameraInfo,rgb);
      m_image_subscriber.clearImage();
    }

    // The information from ROS was copied, this may run along the main thread.
    main_lock.unlock();

    if (hasimage){
      execute(depth,cameraInfo,rgb,pose_hint);
    }

    // lock the mutex again, it will be unlocked by the condition variable at the beginning of the cycle
    main_lock.lock();
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//callback function, called with every new depth topic message
void KinFuDLApp::execute(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo, const sensor_msgs::ImageConstPtr& rgb,const KinfuTracker::THint & pose_hint){
  frame_counter_++;

  if (kinfu_->icpIsLost())
  {
		kinfu_->reset();
	}

  depth_device_.upload (&(depth->data[0]), depth->step, depth->height, depth->width);

  /*
   *      [fx  0 cx]
   * K = 	[ 0 fy cy]
   *			[ 0  0  1]
   */

  (*kinfu_).setDepthIntrinsics(cameraInfo->K[0], cameraInfo->K[4], cameraInfo->K[2], cameraInfo->K[5]);

  float focal_length = (cameraInfo->K[0] + cameraInfo->K[4]) / 2;
  
  SampledScopeTime fps(time_ms_);

	//************** added by DH **************//
	std::vector<Eigen::Vector3f> robot_joints;
	geometry_msgs::Pose RGBD_sensor_pose;		
	bool robot_state = m_robot_subscriber.getRobotJoints(robot_joints, RGBD_sensor_pose);		
	//std::cout << "right_lower_elbow:(" << robot_joints[0].x() << " " << robot_joints[0].y() << " " << robot_joints[0].z() << endl;
	//std::cout << "right_lower_forearm:(" << robot_joints[1].x() << " " << robot_joints[1].y() << " " << robot_joints[1].z() << endl;
	//std::cout << "right_hand:(" << robot_joints[2].x() << " " << robot_joints[2].y() << " " << robot_joints[2].z() << endl;
	
	KinfuTracker::THint pose_hint_kinect;
	pose_hint_kinect.type = KinfuTracker::THint::HINT_TYPE_HINT;
	Eigen::Quaternionf Qtmp(RGBD_sensor_pose.orientation.w, RGBD_sensor_pose.orientation.x, RGBD_sensor_pose.orientation.y, RGBD_sensor_pose.orientation.z);
	Eigen::Matrix3f Qmat = Qtmp.toRotationMatrix();

	pose_hint_kinect.transform.translation()[0] = RGBD_sensor_pose.position.x;
	pose_hint_kinect.transform.translation()[1] = RGBD_sensor_pose.position.y;
	pose_hint_kinect.transform.translation()[2] = RGBD_sensor_pose.position.z;
	
	pose_hint_kinect.transform.linear() = Qmat;		

	if (robot_state) {
		(*kinfu_)(depth_device_, robot_joints, pose_hint);
		//else (*kinfu_)(depth_device_, pose_hint);
	  if (kinfu_->isFinished())
	    nh.shutdown();
		
		if (m_command_subscriber.extract_mesh()){
			tf::Vector3 cursor;
			Eigen::Vector3d boxCenter;
			m_phantom_subscriber.getCursor(cursor);

			boxCenter = kinfu_->extractBox(Eigen::Vector3f(cursor.getX(),cursor.getY(),cursor.getZ()));
			//std::vector<float> tsdfBox;																				
			//tsdfBox.clear();
			//kinfu_->volume_box().downloadTsdf(tsdfBox);	
	
			std::vector<Eigen::Vector3d> meshes;	
			m_marching_cube.extractBoxMesh(kinfu_, meshes, true);
			m_mesh_publisher.publishMeshes(meshes, boxCenter, true);
			//usleep(500000);
			//************** added by DH **************//
		}
		else{
			std::vector<Eigen::Vector3d> meshes;	
			Eigen::Vector3d boxCenter;
			m_mesh_publisher.publishMeshes(meshes, boxCenter, true);
		}
	  m_image_publisher.publishScene(*kinfu_,depth);
	  m_pose_publisher.publishPose(*kinfu_);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void KinFuDLApp::start()
{
  m_thread = boost::thread(&KinFuDLApp::run,this);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void KinFuDLApp::prepareTermination()
{
  boost::mutex::scoped_lock lock(m_mutex);
  m_request_termination = true;
  m_cond.notify_one();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void KinFuDLApp::join()
{
  prepareTermination();
  m_thread.join();
}


