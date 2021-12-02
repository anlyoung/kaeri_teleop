#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_listener.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <ncurses.h>

#include <string>
#include <sstream>                      //istringstream, ostringstream
#include <fstream>                      //file write load
#include <vector>                       //vector

using namespace std;

class omni_pub{


private:
  ros::NodeHandle nh;
  int publish_freq;

  tf::TransformListener listener;
  ros::Publisher mesh_pub;
  ros::Publisher STL_update_pub;
  ros::Subscriber STL_update_sub;

  vector< vector<tf::Vector3> > meshBuffer;

  tf::StampedTransform tfMesh2Phantom;
  tf::StampedTransform tfMesh2Phantom_save;
  bool newMesh;

public:
  omni_pub(){

    mesh_pub = nh.advertise<std_msgs::Float64MultiArray>("Phantom_mesh", 100);
    STL_update_sub = nh.subscribe("obj_update", 1, &omni_pub::updateSTLCallback, this);
    newMesh = true;
    nh.param(std::string("publish_rate"), publish_freq, 10);
  }

  void updateSTLCallback(const std_msgs::Int32MultiArray::ConstPtr& msg){
      meshBuffer.clear();
      for(int i=0;i<msg->layout.dim[0].size;i++){
          meshBuffer.push_back(readSTL(msg->data[i]));
      }
      newMesh = true;
      ROS_INFO("%s", "new mesh read!");
  }

  vector<tf::Vector3> readSTL(int id){
      ifstream loadFile;
      istringstream iss;
      ostringstream oss;
      string input, dummy1, dummy2;

      vector<tf::Vector3> Buffer;
      oss<<"/home/anl-testbed/catkin_ws/src/rviz_controller/objects/ascii_"<<id<<".stl";
      string filename = oss.str();
      const char * fname = filename.c_str();
      loadFile.open (fname, ifstream::in);

      // read header
      getline(loadFile, dummy1);
      ROS_INFO("%s", "Reading STL file");
      tf::Vector3 vTmp;
      double x, y, z;
      while (loadFile.good()){
          // parsing normal line
          getline(loadFile, dummy1);
          if (dummy1 == "endsolid Mesh") break;
          // parsing vertices
          getline(loadFile, dummy1);
          for (int i=0; i<3; i++){
              getline(loadFile, input);
              iss.str(input);
              iss >> dummy2 >> x >> y >> z;
              iss.clear();
              vTmp = tf::Vector3(x, y, z);
              Buffer.push_back(vTmp);
          }
          // parsing normal line
          getline(loadFile, dummy1);
          // parsing vertices
          getline(loadFile, dummy1);
      }
      int n = Buffer.size();
      ROS_INFO("%s %d %s", "STL file read", n, "vertices");
      return Buffer;
  }

  void meshTransform(std_msgs::Float64MultiArray *msg){
      int n = meshBuffer.size();
      int totalVtxNumber = 0;
      for (int i=0;i<n;i++){
          totalVtxNumber = totalVtxNumber + meshBuffer[i].size();
      }
      msg->layout.dim[0].label = "nVtx";
      msg->layout.dim[1].label = "coordinate";
      msg->layout.dim[0].size = totalVtxNumber;
      msg->layout.dim[0].stride = totalVtxNumber*3;
      msg->layout.dim[1].size = 3;
      msg->layout.dim[1].stride = 3;
      msg->data.clear();

      tf::Vector3 vTmp;

			tf::Vector3 o = tfMesh2Phantom.getOrigin();
    	tf::Quaternion q = tfMesh2Phantom.getRotation();	
			//std::cout << o.getX() << " " << o.getY() << " " << o.getZ() << std::endl;
			//std::cout << q.getX() << " " << q.getY() << " " << q.getZ() << " " << q.getZ() << std::endl;

      for (int i=0;i<n;i++){
          int nvtx = meshBuffer[i].size();
          for (int j=0;j<nvtx;j++){
							//if (i == 0) std::cout << j << ": " << meshBuffer[i][j].getX() << " " << meshBuffer[i][j].getY() << " " << meshBuffer[i][j].getZ() << std::endl; 
              vTmp = tfMesh2Phantom(meshBuffer[i][j]);
              msg->data.push_back(vTmp.getX());
              msg->data.push_back(vTmp.getY());
              msg->data.push_back(vTmp.getZ());
							//if (i == 0) std::cout << j << ": " << msg->data[i*3]*2 << " " << msg->data[i*3+1]*2 << " " << msg->data[i*3+2]*2 << std::endl; 
              //ROS_INFO("msg: %f %f %f", msg->data[i*3], msg->data[i*3+1], msg->data[i*3+2]);
          }
      }

  }

  void listenMesh2Phantom(){
      // get transform from "Mesh" frame to "Phantom" frame
      listener.waitForTransform("/phantom", "/world", ros::Time::now(), ros::Duration(3.0));
      try{
          listener.lookupTransform("/phantom", "/world", ros::Time(0), tfMesh2Phantom);
      }
      catch(tf::TransformException ex){
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
      }


      meshBuffer.clear();
			for(int i=0;i<3;i++){
      		meshBuffer.push_back(readSTL(i));
			}
			newMesh = true;
      tfMesh2Phantom_save = tfMesh2Phantom;
      ROS_INFO("%s", "mesh moved!");
			/*
      tf::Vector3 o = tfMesh2Phantom.getOrigin();
      tf::Vector3 os = tfMesh2Phantom_save.getOrigin();
      tf::Quaternion q = tfMesh2Phantom.getRotation();
      tf::Quaternion qs = tfMesh2Phantom_save.getRotation();
      bool ori_comp = compare(o.getX(), os.getX()) && compare(o.getY(), os.getY()) && compare(o.getZ(), os.getZ());
      bool rot_comp = compare(q.x(), qs.x()) && compare(q.y(), qs.y()) && compare(q.z(), qs.z()) && compare(q.w(), qs.w());
      if(!ori_comp||!rot_comp){
          ROS_INFO("%f %f %f %f %f %f %f", o.getX(), o.getY(), o.getZ(), q.x(), q.y(), q.z(), q.w());
          ROS_INFO("%f %f %f %f %f %f %f", os.getX(), os.getY(), os.getZ(), qs.x(), qs.y(), qs.z(), qs.w());
          ROS_INFO("%d %d %d", o.getX()==os.getX(), o.getY()==os.getY(), o.getZ()==os.getZ());
          ROS_INFO("%d %d %d %d", q.x()==qs.x(), q.y()==qs.y(), q.z()==qs.z(), q.w()==qs.w());
          ROS_INFO("%d %d", ori_comp, rot_comp);
          newMesh = true;
          tfMesh2Phantom_save = tfMesh2Phantom;
          ROS_INFO("%s", "mesh moved!");
      }
*/
  }

  void publish_all(std_msgs::Float64MultiArray *msg){
      int nVtx;
      nVtx = msg->layout.dim[0].size;
      mesh_pub.publish(*msg);
      ROS_INFO("%d vertices published!", nVtx);

      newMesh = false;
  }

  int publish_rate(){
      return publish_freq;
  }

  bool meshUpdate(){
      return newMesh;
  }

  bool compare(double a, double b){
      if (fabs(a-b)<0.001) return true;
      else return false;
  }
};

int main(int argc, char** argv) {
    ros::init(argc,  argv, "omni_pub");
    omni_pub opub;

    ros::Rate loop_rate(opub.publish_rate());

    std_msgs::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.data_offset = 0;

    int i = 0;
    while(ros::ok()){
        opub.listenMesh2Phantom();
        ros::spinOnce();

        // newMesh update
        if(opub.meshUpdate()){
            opub.meshTransform(&msg);
            opub.publish_all(&msg);
            ROS_INFO("%s","new mesh generated");
            i = 0;
        }

        i++;
        loop_rate.sleep();
    }
    return 0;
}
