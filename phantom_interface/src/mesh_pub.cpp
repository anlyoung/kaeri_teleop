// ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

// Openhaptics
#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

// Linux
#include <ncurses.h>

// STL
#include <stdio.h>
#include <string>
#include <assert.h>
#include <sstream>                      //istringstream, ostringstream
#include <fstream>                      //file write load
#include <vector>                       //vector

// Boost
#include <boost/thread/mutex.hpp>
using namespace std;

class omni_pub{

private:
  ros::NodeHandle m_nh;
  int publish_freq;

  tf::TransformListener listener;
  ros::Publisher mesh_pub;
	ros::Subscriber RGBD_mesh_sub;
	ros::Subscriber vf_mesh_sub;
	ros::Subscriber STL_mesh_sub;

  vector<tf::Vector3> vf_mesh_buffer;
	vector<tf::Vector3> RGBD_mesh_buffer;
	std::string RGBD_mesh_frame;	

  tf::StampedTransform tfWorld2Phantom;
  tf::StampedTransform tfDepth2Phantom;

  bool newMesh;
	boost::mutex &m_tf_mutex;
	boost::mutex &m_mesh_mutex;

public:
  omni_pub(ros::NodeHandle &nh, boost::mutex &tf_mutex, boost::mutex &mesh_mutex): m_nh(nh), m_tf_mutex(tf_mutex), m_mesh_mutex(mesh_mutex)
	{
    RGBD_mesh_frame = " ";
    mesh_pub = m_nh.advertise<geometry_msgs::PoseArray>("Phantom_mesh", 1);
    vf_mesh_sub = m_nh.subscribe("vf_objects", 1, &omni_pub::vf_mesh_callback, this);
    RGBD_mesh_sub = m_nh.subscribe("kinfu_mesh_topic", 1, &omni_pub::RGBD_mesh_callback, this);
		STL_mesh_sub = m_nh.subscribe("VF_Fixture",1, &omni_pub::STL_Mesh_callback,this);

    newMesh = true;
  }

  void vf_mesh_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){
			boost::mutex::scoped_lock lock(m_mesh_mutex);
      vf_mesh_buffer.clear();
      for(int i = 0; i < (int)msg->markers.size(); i++){
					for(int j = 0; j < (int)msg->markers[i].points.size(); j++){
							tf::Vector3 vtx(msg->markers[i].points[j].x, msg->markers[i].points[j].y, msg->markers[i].points[j].z);
				    	vf_mesh_buffer.push_back(vtx);
							//if (i == 0) std::cout << j << ": " << msg->markers[i].points[j].x << " " << msg->markers[i].points[j].y << " " << msg->markers[i].points[j].z << std::endl; 
					}
      }
  }

	void RGBD_mesh_callback(const visualization_msgs::Marker::ConstPtr& msg){			
			RGBD_mesh_frame = msg->header.frame_id;
			boost::mutex::scoped_lock lock(m_mesh_mutex);
      RGBD_mesh_buffer.clear();
      for(int i = 0; i < (int)msg->points.size()/3; i++){
					tf::Vector3 vtx0(msg->points[3*i].x, msg->points[3*i].y, msg->points[3*i].z);				
					tf::Vector3 vtx1(msg->points[3*i+1].x, msg->points[3*i+1].y, msg->points[3*i+1].z);
					tf::Vector3 vtx2(msg->points[3*i+2].x, msg->points[3*i+2].y, msg->points[3*i+2].z);
		    	RGBD_mesh_buffer.push_back(vtx0);
		    	RGBD_mesh_buffer.push_back(vtx1);
		    	RGBD_mesh_buffer.push_back(vtx2);
		    	//RGBD_mesh_buffer.push_back(vtx0);
		    	//RGBD_mesh_buffer.push_back(vtx2);
		    	//RGBD_mesh_buffer.push_back(vtx1);
	    }
  }

	// Krebs
	void STL_Mesh_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
boost::mutex::scoped_lock lock(m_mesh_mutex);
      RGBD_mesh_buffer.clear();
      for(int i = 0; i < (int)msg->poses.size(); i++){
					tf::Vector3 vtx0(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);	
					i++;		
					tf::Vector3 vtx1(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);	
					i++;			
					tf::Vector3 vtx2(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);				

					if (vtx0[0] > .1) {
						std::cout << "Point X: " << vtx0[0] << std::endl;
					}
					
		    	RGBD_mesh_buffer.push_back(vtx0);
		    	RGBD_mesh_buffer.push_back(vtx1);
		    	RGBD_mesh_buffer.push_back(vtx2);
		    	//RGBD_mesh_buffer.push_back(vtx0);
		    	//RGBD_mesh_buffer.push_back(vtx2);
		    	//RGBD_mesh_buffer.push_back(vtx1);
	    }
	}
	
	void tf_listener(){
			boost::mutex::scoped_lock lock(m_tf_mutex);
      if(listener.waitForTransform("/phantom", "/world", ros::Time::now(), ros::Duration(0.1))){
				  try{
				      listener.lookupTransform("/phantom", "/world", ros::Time(0), tfWorld2Phantom);
				  }
				  catch(tf::TransformException ex){
				      ROS_ERROR("%s", ex.what());
				  }
			}
      else std::cout << "cannot find tf from world to phantom" << std:: endl;

			if(listener.waitForTransform("/phantom", RGBD_mesh_frame, ros::Time::now(), ros::Duration(0.1))){
				  try{
				      listener.lookupTransform("/phantom", RGBD_mesh_frame, ros::Time(0), tfDepth2Phantom);
				  }
				  catch(tf::TransformException ex){
				      ROS_ERROR("%s", ex.what());
				  }
			}
      else std::cout << "cannot find tf from " << RGBD_mesh_frame << " to phantom" << std:: endl;
  }

  void transfrom_and_publish(){
			boost::mutex::scoped_lock tf_lock(m_tf_mutex);
			boost::mutex::scoped_lock mesh_lock(m_mesh_mutex);
	 
			geometry_msgs::PoseArray meshes;
      tf::Vector3 o = tfWorld2Phantom.getOrigin();
    	tf::Quaternion q = tfWorld2Phantom.getRotation();	
			//std::cout << o.getX() << " " << o.getY() << " " << o.getZ() << std::endl;
			//std::cout << q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getZ() << std::endl;
			for (int i = 0; i < (int)vf_mesh_buffer.size(); i++){
					//if (i < 120) std::cout << i << ": " << vf_mesh_buffer[i].getX() << " " << vf_mesh_buffer[i].getY() << " " << vf_mesh_buffer[i].getZ() << std::endl; 
          tf::Vector3 vTmp = tfWorld2Phantom(vf_mesh_buffer[i]);
					geometry_msgs::Pose pt;
					pt.position.x = vTmp.getX();
					pt.position.y = vTmp.getY(); 
					pt.position.z = vTmp.getZ();
					
					//if (i < 120) std::cout << i << ": " << pt.x*2 << " " << pt.y*2 << " " << pt.z*2 << std::endl; 
					meshes.poses.push_back(pt);
			}

			for (int i = 0; i < (int)RGBD_mesh_buffer.size(); i++){
          tf::Vector3 vTmp = tfDepth2Phantom(RGBD_mesh_buffer[i]);
					geometry_msgs::Pose pt;
					pt.position.x = vTmp.getX();
					pt.position.y = vTmp.getY(); 
					pt.position.z = vTmp.getZ();
					meshes.poses.push_back(pt);
					if (pt.position.x > .1) {
						std::cout << "Point X: " << pt.position.x << std::endl;
					}
			}
			std::cout << "Meshes Size: " << meshes.poses.size() << std::endl;
			mesh_pub.publish(meshes);
			//ROS_INFO("%d vertices are published\n", int(meshes.poses.size()));
  }
};

int main(int argc, char** argv) {
    ros::init(argc,  argv, "omni_pub");
		ros::NodeHandle nh;

		boost::mutex tf_mutex;
		boost::mutex mesh_mutex;
		
		ros::Rate loop_rate(20);
    omni_pub opub(nh, tf_mutex, mesh_mutex);

    while(ros::ok()){
        ros::spinOnce();
        opub.tf_listener();				
				opub.transfrom_and_publish();
        loop_rate.sleep();
    }
    return 0;
}
