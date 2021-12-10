#ifndef SAWYERMOTIONPRIMITIVES_H
#define SAWYERMOTIONPRIMITIVES_H
// https://github.com/wsnewman/ros_class/blob/master/example_ros_class/src/example_ros_class.cpp

#include "pbdlib/trajgmm.h"
#include "pbdlib/gmm.h"
#include <armadillo>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "intera_core_msgs/EndpointState.h"

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
    int ButtonState;
    bool Record;
};

enum sVirualPoseRecStatus
{  
    RECORDING = 1,                  // 1 (Button Dragging)
    JUST_STARTED,                   // 2 (Button Down)
    JUST_FINSHED                    // 3 (Button Up)
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

enum sTrainDataType
{
    RECORD_CURRENT_VIRTUAL_DATA,                    // 0
    USE_TESTDATA_VIRTUAL_CIRCLE,                    // 1
    USE_TESTDATA_DRAW_U_MOTION,                     // 2
    USE_TESTDATA_DRAW_V_MOTION,                     // 3
    SIZE_OF_TRAINDATA_TYPE                          // 4   
};



class SawyerMotionPrimitives
{
private:
    int     m_nCurMotionCnt;
    int     m_nMaxMotionCnt;
    int     m_bIsTraining;
    bool    m_bSaveFileFlag;
    bool    m_bUseTestTrainDataFlag;   // See if training procedure is skipeed --> then load defaultdata
public:
    sRobotXYZCoord  Origin;
    sRobotXYZWCoord Bottom;
    
    sRobotXYZCoord StartPt;

public:
    SawyerMotionPrimitives(ros::NodeHandle* nodehandle);        // Constructor
    void InitializeSubscribers();
    void InitializePublishers();
    void InitializeServices();

    // Methods
    void StartDemonstration();
    void TrainingPhantomData();
    void TrainingUnityData();
    void MakeTrajectoryAlogrithm();

    void SendCmdMoveEndPointwithTrajectory();
    void SendCmdMove(sRobotEndPointPose pose);
    void SendCmdMoveDemoPosition();

    sRobotEndPointPose GetCurRobotEndPos();
    sRobotEndPointPose GetTargetRobotEndPos();
    sRobotEndPointPose GetPrevRobotEndPos();
    sRobotEndPointTrajectory GetPrevRobotEndTrajectories();

    // Sets
    void SetMaxRepeatMotions(int iMaxVal);
    void SetGMMParams(sGMMParams params);
    void SetGMMParams(int nbStates, int nbVarPos, int nbDeriv, double dt);
    void SetDataSize(int nDataSize);
    void SetSaveFile(bool bSave);
    void SetUseTestTrainDataFiles(int nType);

    // Gets
    int GetCurMotionCnt();
    bool IsTraining();
    sGMMParams GetGMMParams();
    
    // Callbacks
    void PhantomJointCallback(const geometry_msgs::Pose::ConstPtr &msg);
    void PhantomButtonCallback(const std_msgs::String &msg);
    void RobotJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void RobotEndPointCallback(const intera_core_msgs::EndpointState::ConstPtr &msg);

    void UnityRightHandControllerPositionCallback(const geometry_msgs::Pose::ConstPtr &msg);
    void UnityRightHandControllerButtonStatusCallback(const std_msgs::Int32 &msg);

    TrajGMM *pPhantomTrajGMM1;

    std::string basePath;
    std::string trainDataPath;

    sVirtualPoseData        curPhantomData;
    sVirtualPoseData        curUnityData;
    sGMMParams              curGMMparams;
    int                     curTrainDataType;

    sRobotEndPointPose              m_vRobotEndPt;              // Robot End Point
    sRobotEndPointPose              m_vRobotTargetEndPt;        // Robot End Point
    sRobotEndPointPose              m_vRobotPrevPt;             // Robot End Point
    sRobotEndPointTrajectory        m_vRobotPrevTrajectories;

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
    ros::Subscriber sawyer_joint_sub;
    ros::Subscriber sawyer_endpoint_sub;

     // From the Unity
    ros::Subscriber unity_position_sub;
    ros::Subscriber unity_button_status_sub;


	ros::Publisher trajgmm_posArr_pub;
};

#endif
