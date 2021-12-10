/*
--------------------------------------------------------------------------------------------------
Author : Dongjune Chang (dongjune.chang@gmail.com) 
Date   : June, 24. 2019

This is one of the example programs of motion primitives based on the data from unity-oculus hand controller
and phantom omni device. 
In order to extract the high expectation of the optimal path, the TrajGMM class from PbdLib, 
the reproduction of trajectory with a GMM with dynamic features (trajectory-GMM) is used for main training algorithm.

(1) i-th Motion Demonstration data files location : trainDataPath + (i-1).txt
(2) motion primitives data file after calculation : trainDataPath + optimalData1.txt

--------------------------------------------------------------------------------------------------
PbDLib is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

PbDLib is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with PbDLib.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOTIONPRIMITIVES_H
#define MOTIONPRIMITIVES_H

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
//#include <example_srv/simple_bool_service_message.h> // this is a pre-defined service message, contained in shared "example_srv" package
#include </usr/include/armadillo>
#include "pbdlib/trajgmm.h"
#include "pbdlib/gmm.h"

using namespace arma;
using namespace pbdlib;

typedef geometry_msgs::Point        sRobotXYZCoord;
typedef geometry_msgs::Quaternion   sRobotXYZWCoord;         
typedef geometry_msgs::Pose         sRobotEndPointPose;
typedef geometry_msgs::PoseArray    sRobotEndPointTrajectory;

struct sVirtualPoseData
{
    float X;
    float Y;
    float Z;
    float qX;
    float qY;
    float qZ;
    float qW;
    int ButtonState;
    bool Record;
};

enum sVirtualPoseRecStatus
{   
    FREE_STATE = 0,                 // 0 (Free State)
    JUST_STARTED,                   // 1 (Button Dragging)
    RECORDING,                      // 2 (Button Down)             
    JUST_FINSHED                    // 3 (Button Up)
};

enum sVirtualDeviceMode
{
    MODE_PHANTOM_OMNI_DEVICE = 0,
    MODE_OCULUS_HAND_CONTROLLER        // Requre 6 joints
};

enum sTrainDataType
{
    RECORD_CURRENT_VIRTUAL_DATA,                    // 0
    USE_TESTDATA_VIRTUAL_CIRCLE,                    // 1
    USE_TESTDATA_DRAW_U_MOTION,                     // 2
    USE_TESTDATA_DRAW_V_MOTION,                     // 3
    SIZE_OF_TRAINDATA_TYPE                          // 4   
};

struct sGMMParams
{
    int nbStates;		        // Number of components in the GMM
    int nbVarPos;		        // Dimension of position data (here: x1,x2)
    int nbDeriv;		        // Number of static&dynamic features (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
    double dt;                  // Time step (without rescaling, large values such as 1 has the advantage of creating clusers based on position information)
    int nbData;	                // Number of  datapoints in a trajectory
    int nbSamples;  	        // Number of demonstrations
};

class   MotionPrimitives
{
private:
    int                              m_nCurMotionCnt;
    int                              m_nMaxMotionCnt;
    int                              m_bIsTraining;
    bool                             m_bSaveFileFlag;
    bool                             m_bUseTestTrainDataFlag;   // See if training procedure is skipeed --> then load defaultdata

    sVirtualPoseRecStatus            m_sPhantomStatus[2];            //    [0] : Current,  [1] : Last
    sVirtualPoseRecStatus            m_sUnityStatus[2];              //    [0] : Current,  [1] : Last
public:
    sRobotXYZCoord  Origin;
    sRobotXYZWCoord Bottom;
    
    sRobotXYZCoord StartPt;

public:
    MotionPrimitives(ros::NodeHandle* nodehandle);        // Constructor
    void InitializeSubscribers();
    void InitializePublishers();
    void InitializeServices();

    // Methods
    void StartDemonstration();
    bool TrainingPhantomData();
    bool TrainingUnityData();
    bool MakeTrajectoryAlogrithm();

    // Sets
    void SetMaxRepeatMotions(int iMaxVal);
    void SetGMMParams(sGMMParams params);
    void SetGMMParams(int nbStates, int nbVarPos, int nbDeriv, double dt);
    void SetDataSize(int nDataSize);
    void SetSaveFile(bool bSave);
    void SetUseTestTrainDataFiles(int nType);
    void SetDeviceType(int nType);

    // Gets
    int GetCurMotionCnt();
    bool IsTraining();
    sGMMParams GetGMMParams();
    
    // Callbacks
    void PhantomJointCallback(const geometry_msgs::Pose::ConstPtr &msg);
    void PhantomButtonCallback(const std_msgs::String &msg);
    void UnityRightHandControllerPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void UnityRightHandControllerButtonStatusCallback(const std_msgs::Int32 &msg);

    //bool serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
    bool GetOptimalTrajectory(mat &OptimalMatData);
    int GetVirtualDeviceType();
    void PrintOptimalData();   

    std::string basePath;
    std::string trainDataPath;

    sVirtualDeviceMode      curVirtualDeviceType;
    sVirtualPoseData        curPhantomData;
    sVirtualPoseData        curUnityData;
    sGMMParams              curGMMparams;
    int                     curTrainDataType;

    std::vector<double> m_vPhantomX;
    std::vector<double> m_vPhantomY;
    std::vector<double> m_vPhantomZ;

    std::vector<double> m_vUnityX;
    std::vector<double> m_vUnityY;
    std::vector<double> m_vUnityZ;

    std::vector<mat> m_mDemoTrainData;
    mat m_mDemoOptimalData;

    ros::NodeHandle nh_;

    ros::Subscriber phantom_joint_sub;
    ros::Subscriber phantom_button_sub;

     // From the Unity
    ros::Subscriber unity_position_sub;
    ros::Subscriber unity_button_status_sub;

	ros::Publisher trajgmm_posArr_pub;
};

#endif
