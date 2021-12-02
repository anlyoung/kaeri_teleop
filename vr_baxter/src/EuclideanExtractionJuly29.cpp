#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"
//#include "VR_Baxter/FloatArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>

#include <vector>

typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
std::vector<float> v;
geometry_msgs::Pose cylinder_Pose;
sensor_msgs::PointCloud2 newPointCloud;
std::vector<float *>point;

typedef Matrix<float, 3,3> Matrix3x3;
typedef Matrix<float, 3,6> Matrix3x6;
typedef Matrix<float, 6,6> Matrix6x6;

//typedef VectorXf<float,6> Vector6f;

void Preprocess (int n, std::vector<Vector3f> points, Vector3f X[], Vector3f& average ,  VectorXf& mu,  Matrix3x3& F0 ,  Matrix3x6& F1 ,  Matrix6x6&F2) {

    average << 0,0,0;

    for (int i =0;i<n;++i)
        average+=points[i];

    average/=(float)n;
   // std::cout << "Average: " << average << std::endl;

    for (int i =0;i<n;++i)
        X[i] = points[i]-average;

    VectorXf zero(6);
    zero << 0,0,0,0,0,0;

    VectorXf fillProducts(6);
    VectorXf products[n];

    mu << 0,0,0,0,0,0;

    for (int i =0;i<n;++i) {
        fillProducts << 0,0,0,0,0,0;
        products[i] = fillProducts;
        products[i][0] = X[i][0] * X[i][0];
        products[i][1] = X[i][0] * X[i][1];
        products[i][2] = X[i][0] * X[i][2];
        products[i][3] = X[i][1] * X[i][1];
        products[i][4] = X[i][1] * X[i][2];
        products[i][5] = X[i][2] * X[i][2];

        mu[0] += products[i][0];
        mu[1] += 2* products[i][1];
        mu[2] += 2* products[i][2];
        mu[3] += products[i][3];
        mu[4] += 2*products[i][4];
        mu[5] += products[i][5];
    }
    mu/=(float)n;

    F0 *= 0;
    F1 *= 0;
    F2 *= 0;

    for (int i = 0; i<n; ++i) {
        VectorXf delta(6);
        delta[0] = products[i][0];
        delta[1] = 2 * products[i][1];
        delta[2] = 2 * products[i][2];
        delta[3] = products[i][3];
        delta[4] = 2 * products[i][4];
        delta[5] = products[i][5];
        F0.row(0)[0] += products[i][0];
        F0.row(0)[1] += products[i][1];
        F0.row(0)[2] += products[i][2];
        F0.row(1)[1] += products[i][3];
        F0.row(1)[2] += products[i][4];
        F0.row(2)[2] += products[i][5];

        for (int x =0 ;x<3;x++) {
            for (int y = 0;y<6;y++) {
                F1.row(x)[y] = X[i][x]*delta[y];
            }
        }
        for (int x =0 ;x<6;x++) {
            for (int y = 0;y<6;y++) {
                F2.row(x)[y] = delta[x]*delta[y];
            }
        }
    }
    F0 /=(float) n;
    F0.row(1)[0] = F0.row(0)[1];
    F0.row(2)[0] = F0.row(0)[2];
    F0.row(2)[1] = F0.row(1)[2];
    F1/=(float)n;
    F2/=(float)n;
}

float G(int n,  Vector3f X[] ,  VectorXf& mu,  Matrix3x3&  F0 ,  Matrix3x6&  F1 ,  Matrix6x6&  F2 ,Vector3f& W,Vector3f& PC,  float& rSqr)
{
    Matrix3x3 P = Matrix3x3::Identity();

    Matrix3x3 outProd;

    for (int x =0 ;x<3;x++) {
        for (int y = 0;y<3;y++) {
            outProd.row(x)[y] = W(x)*W(y);
        }
    }

    P -= outProd;

    Matrix3x3 SA;
    SA << 0,-W[2],W[1],W[2],0,-W[0],-W[1],W[0],0;
    Matrix3x3 A=P*F0*P;
    Matrix3x3 hatA = -(SA*A*SA);
    Matrix3x3 hatAA = hatA * A;
    float trace = hatAA.trace();
    Matrix3x3 Q = hatA/trace;
    VectorXf p(6);
    p << P(0),P(1),P(2),P(4),P(5),P(8);
    Vector3f alpha = F1 * p;
    Vector3f beta = Q * alpha;
    float error = (p.dot(F2*p)-4*alpha.dot(beta)+4*beta.dot(F0*beta))/n;
    PC = beta;
    rSqr = p.dot(mu)+beta.dot(beta);

    return error;
}

float FitCylinder(int n, std::vector<Vector3f> points, float& rSqr, Vector3f& C, Vector3f& W)
{
    Vector3f X[n];
    VectorXf mu(6);
    Matrix3x3  F0;
    Matrix3x6  F1;
    Matrix6x6  F2;
    Vector3f average(0,0,0);

    Preprocess(n, points, X, average, mu, F0, F1, F2);

   // std::cout << "AFTER PREPROCESS:" << std::endl;
  //  std::cout << "mu: " << mu << std::endl;
   // std::cout << "F0: " << F0<< std::endl;
   // std::cout << "F1: " << F1 << std::endl;
   // std::cout << "F2: " << F2 << std::endl;



    float minError = 999;
    rSqr = 0;

    // IMAX & JMAX is desired level of granularity
    float jmax = 100;
    float imax = 100;

    float halfPi = 1.5707963;
    float twoPi = 6.2831853;

    float error = 0;

    for (float j = 0;j<=jmax;++j) {
        float phi = halfPi * (j/jmax);
        float csphi = cos(phi);
        float snphi = sin(phi);

        for (float i= 0; i<imax;++i) {
            float theta = twoPi * (i/imax);
            float cstheta = cos(theta);
            float sntheta = sin(theta);

            Vector3f currentW(cstheta*snphi, sntheta*snphi,csphi);
            Vector3f currentC(0,0,0);
            float currentRSqr;
            error = G(n,X,mu,F0,F1,F2,currentW,currentC,currentRSqr);
            if (error <minError) {
                minError = error;
                W = currentW;
                C = currentC;
                rSqr = currentRSqr;
                //std::cout << "rSqr: " << sqrt(currentRSqr)/100 << std::endl;
                //std::cout << "Error: " <<error << std::endl;
            }
        }
    }

    C[0] += average[0];
    C[1] += average[1];
    C[2] += average[2];
    return minError;
}


void extractPC(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // All the objects needed
  pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  //reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
  pcl::fromROSMsg(*msg,*cloud);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);


  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.2);
  seg.setRadiusLimits (0, 0.2);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
      pcl::toROSMsg(*cloud_cylinder.get(),newPointCloud);
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
  }
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    extractPC(msg);
}

void processCloud() {
    float max_y = 0;
    float min_y = 99;
    float max_x = 0;
    float min_x = 99;
    float max_z = 0;
    float min_z = 99;
    float dif_x = 0;
    float dif_y = 0;
    float dif_z = 0;
    float arr[] = {0,0,0};
    v.clear();
                int count = 0;


    std::vector<Vector3f> point;
    for (int x = 0; x<cloud_cylinder->points.size(); x++) {
//        if (!isnan(cloud.points[x].x) && cloud.points[x].z <2 && cloud.points[x].z >.5f && x%30==n) {
        if (!isnan(cloud_cylinder->points[x].x) &&!isnan(cloud_cylinder->points[x].y)&&!isnan(cloud_cylinder->points[x].z)) {

        //if (!isnan(cloud.points[x].x) && x%10 == n) {
            if (cloud_cylinder->points[x].x > max_x)
                max_x = cloud_cylinder->points[x].x;
            else if (cloud_cylinder->points[x].x < min_x)
                min_x = cloud_cylinder->points[x].x;

            if (cloud_cylinder->points[x].y > max_y)
                max_y = cloud_cylinder->points[x].y;
            else if (cloud_cylinder->points[x].y < min_y)
                min_y = cloud_cylinder->points[x].y;

            if (cloud_cylinder->points[x].z > max_z)
                max_z = cloud_cylinder->points[x].z;
            else if (cloud_cylinder->points[x].z < min_z)
                min_z = cloud_cylinder->points[x].z;

            v.push_back((float)cloud_cylinder->points[x].x);
            v.push_back((float)cloud_cylinder->points[x].y);
            v.push_back((float)cloud_cylinder->points[x].z);
            v.push_back((float)cloud_cylinder->points[x].r);
            v.push_back((float)cloud_cylinder->points[x].g);
            v.push_back((float)cloud_cylinder->points[x].b);
           
            Vector3f newPoint((float)cloud_cylinder->points[x].x * 100,(float)cloud_cylinder->points[x].y*100,(float)cloud_cylinder->points[x].z*100);
            point.push_back(newPoint);
        }
    }
   // std::cout << "COUNT: " << count <<std::endl;

    //std::cout << "MIN_Z: " << min_z << "MAX_Z: " << max_z << std::endl;
    float rSqr;
    Vector3f C(0,0,0);
    Vector3f W(0,0,0);
    float error = 0;

    if (cloud_cylinder->points.size() >0)
        error =FitCylinder(point.size(),point, rSqr,C,W);

   // std::cout << "rSqr: " << sqrt(rSqr)/100 << std::endl;
  //  std::cout << "Error: " << error << std::endl;
    //point.clear();
    float newRad = sqrt(rSqr)/100;

    dif_x = max_x-min_x;
    dif_y = max_y-min_y;
    dif_z = max_z-min_z;

    float pos_x = min_x+newRad;
    float pos_y = min_y+newRad;
    float pos_z = min_z+newRad;
    float radius = (dif_x + dif_z)/2.0;

    geometry_msgs::Point cylinder_pos;
    geometry_msgs::Quaternion cylinder_size;

    cylinder_pos.x = C[0]/100.0;
    cylinder_pos.y = C[1]/100.0;
    cylinder_pos.z = C[2]/100.0;

    cylinder_size.x = newRad;
    cylinder_size.y = dif_y;
    cylinder_size.z = dif_z;
    cylinder_size.w = 0.0f;

    cylinder_Pose.position = cylinder_pos;
    cylinder_Pose.orientation = cylinder_size;

    //std::cout << "dif_X: " << cylinder_pos.x<< std::endl;
  //  std::cout << "dif_y: " <<  dif_y<< std::endl;
  //  std::cout << "dif_z: " <<  dif_z<< std::endl;
  //  std::cout << "Radius: " << newRad << std::endl;
  //  std::cout << "pos_X: " <<  cylinder_pos.x<< std::endl;
   // std::cout << "pos_y: " <<  cylinder_pos.y<< std::endl;
   // std::cout << "pos_z: " <<  cylinder_pos.z<< std::endl;

    //std::cout << v.size() << std::endl;
    //for (int x = 0;x<point.size();x++) {
    //    delete[] point[x];
   // }
}

int
main (int argc, char** argv)
{
  ros::init (argc,argv, "PointCloudExtraction");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/kinect/depth_registered/points",1,callback);
    //ros::Subscriber sub = n.subscribe("/xtion/depth_registered/points",1,callback);
    //ros::Publisher pub = n.advertise<VR_Baxter::FloatArray>("/VR_PointCloud",1);
    ros::Publisher pos_pub = n.advertise<geometry_msgs::Pose>("/VR_Cylinder_Pos",1);
    ros::Publisher pubPointCloud = n.advertise<sensor_msgs::PointCloud2>("/Cylinder_PointCloud",1);


    ros::Rate loop_rate(30);

    while (ros::ok()) {
        //VR_Baxter::FloatArray arr;
        processCloud();
				

        //arr.data.clear();

      		//std::cout<< cylinder_Pose.orientation.x<<std::endl;
				
				/*if (cylinder_Pose.orientation.x<.1){
			  for (int i=0;i<v.size();i++) {
            arr.data.push_back(v.at(i));
        }*/
				//pub.publish(arr);
				pubPointCloud.publish(newPointCloud);
				std:: cout<< "t"<<std::endl;
				
				}
				else {sub.shutdown();}
					
			
			pos_pub.publish(cylinder_Pose);
        

        ros::spinOnce();
        loop_rate.sleep();
    }
  return (0);
}
