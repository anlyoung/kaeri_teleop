#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"
#include "VR_Baxter/FloatArray.h"
#include <iostream>

#include <vector>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <stdlib.h>
#include <time.h>

std::vector<float> v;


void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    v.clear();
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    pcl::fromROSMsg(*msg,cloud);

    srand(time(NULL));
    int n;

    float arr[] = {0,0,0};
    for (int x = 0; x<cloud.points.size(); x++) {
        n = rand()%30;
//        if (!isnan(cloud.points[x].x) && cloud.points[x].z <2 && cloud.points[x].z >.5f && x%30==n) {
        if (!isnan(cloud.points[x].x) && cloud.points[x].z <2 && cloud.points[x].z >.5f) {

        //if (!isnan(cloud.points[x].x) && x%10 == n) {
            v.push_back((float)cloud.points[x].x);
            v.push_back((float)cloud.points[x].y);
            v.push_back((float)cloud.points[x].z);
            v.push_back((float)cloud.points[x].r);
            v.push_back((float)cloud.points[x].g);
            v.push_back((float)cloud.points[x].b);
        }
    }

    std::cout << v.size() << std::endl;
}



int main(int argc, char **argv)
{
    ros::init (argc,argv, "PointCloudProcessor");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/kinect/depth_registered/points",1,callback);
    ros::Publisher pub = n.advertise<VR_Baxter::FloatArray>("/VR_PointCloud",1);

    ros::Rate loop_rate(5);

    while (ros::ok()) {
        VR_Baxter::FloatArray arr;

        arr.data.clear();

        for (int i=0;i<v.size();i++) {
            arr.data.push_back(v.at(i));
        }

        pub.publish(arr);

        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}