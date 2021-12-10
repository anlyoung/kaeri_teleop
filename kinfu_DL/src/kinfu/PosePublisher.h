// STL
#include <iostream>

// Eigen
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class PosePublisher
{
  public:
  PosePublisher(ros::NodeHandle & nhandle)
  {
		nhandle.param<std::string>(PARAM_NAME_PREFIX_TOPIC,m_prefix_name,PARAM_DEFAULT_PREFIX_TOPIC);
		m_link_frame = m_prefix_name + "_link";

    m_reverse_initial_transformation = Eigen::Affine3f::Identity();
    m_initial_transformation = Eigen::Affine3f::Identity();						// added by DH
    nhandle.param<std::string>(PARAM_NAME_TF_REFERENCE_FRAME,m_first_frame_name,PARAM_DEFAULT_TF_REFERENCE_FRAME);
    nhandle.param<std::string>(PARAM_NAME_TF_CURRENT_FRAME,m_current_frame_name,PARAM_DEFAULT_TF_CURRENT_FRAME);
  }

  void publishPose(KinfuTracker& kinfu)
  {
    Eigen::Affine3f original_coords = m_reverse_initial_transformation * kinfu.getCameraPose();

    // after this, the z axis is the sensor axis and points forward
    // the x axis is horizontal (points right) and the y axis is vertical (points downward)
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats = original_coords.linear();
    Eigen::Vector3f teVecs = original_coords.translation();

    tf::Transform transform(
        tf::Matrix3x3(erreMats(0,0),erreMats(0, 1),erreMats(0, 2),
            erreMats(1,0),erreMats(1, 1),erreMats(1, 2),
            erreMats(2,0),erreMats(2, 1),erreMats(2, 2)),
        tf::Vector3(teVecs[0], teVecs[1], teVecs[2])
    );

    //m_transform = tf::StampedTransform(transform, ros::Time::now(), m_first_frame_name, m_current_frame_name);
		//m_tf_broadcaster.sendTransform(m_transform);

		//************** added by DH **************//
    Eigen::Vector3f teVecs2 = m_initial_transformation.translation();
		//tf::Transform transform2(tf::Quaternion(0,0,0,1), tf::Vector3(teVecs2[0], teVecs2[1], teVecs2[2]));
		tf::Transform transform2(tf::Quaternion(0,0,0,1), tf::Vector3(0, 0, 0));		

		m_transform_first2current = tf::StampedTransform(transform, ros::Time::now(), m_first_frame_name, m_current_frame_name);
		m_transform_link2first = tf::StampedTransform(transform2, ros::Time::now(), m_link_frame, m_first_frame_name);
		m_tf_broadcaster.sendTransform(m_transform_first2current);
		m_tf_broadcaster.sendTransform(m_transform_link2first);
		//************** added by DH **************//
  }

  void setReverseInitialTransformation(Eigen::Affine3f it){
    m_reverse_initial_transformation = it;
  }

	//************** added by DH **************//
  void setInitialTransformation(Eigen::Affine3f t){
    m_initial_transformation = t;
  }
	//************** added by DH **************//

  std::string getFirstFrameName() const {return m_first_frame_name; }
  std::string getCurrentFrameName() const {return m_current_frame_name; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  private:
  // initial transformation applied, already inverted
  Eigen::Affine3f m_reverse_initial_transformation;
	Eigen::Affine3f m_initial_transformation;							// added by DH

  // for TF frames
	std::string m_prefix_name;
	std::string m_link_frame;
  std::string m_first_frame_name;
  std::string m_current_frame_name;

  tf::TransformBroadcaster m_tf_broadcaster;

  //tf::StampedTransform m_transform;
	tf::StampedTransform m_transform_first2current;				// added by DH
	tf::StampedTransform m_transform_link2first;					// added by DH

  // forbid this constructor
  PosePublisher() {}
};
