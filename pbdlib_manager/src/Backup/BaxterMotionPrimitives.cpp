#include "BaxterMotionPrimitives.h"
#include "spline.h"
#include <fstream>

// https://github.com/ros-industrial/cros/blob/master/resources/cros_testbed/src/std_msgs_talker.cpp

BaxterMotionPrimitives::BaxterMotionPrimitives(ros::NodeHandle* nodehandle)
    :nh_(*nodehandle)
{
    // constructor
    ROS_INFO("in class constructor of BaxterMotionPrimitives");

    InitializeSubscribers();
    InitializePublishers();
    InitializeServices();

    m_nCurMotionCnt = 0;
    m_nMaxMotionCnt = 0;
    m_bIsTraining = false;
    m_bSaveFileFlag = true;
    m_bUseTestTrainDataFlag = false;

    basePath = "/home/user/module_ws/src/pbdlib_manager/";
    trainDataPath = basePath + "Test/Debug/";
    curTrainDataType = RECORD_CURRENT_VIRTUAL_DATA;

    Origin.x = 0.0;
    Origin.y = 0.0;
    Origin.z = 0.0;

    Bottom.x = -.686443;
    Bottom.y = .727163;
    Bottom.z = -.004205;
    Bottom.w = .003220;

    StartPt.x = 0.55048;
    StartPt.y = 0.01047;
    StartPt.z = 0.048439;
    
    m_vRobotEndPt.position = Origin;
    m_vRobotEndPt.orientation = Bottom;

    m_vRobotTargetEndPt.position = Origin;
    m_vRobotTargetEndPt.orientation = Bottom;

    m_vRobotPrevPt.position = Origin;
    m_vRobotPrevPt.orientation = Bottom;

    m_vRobotPrevTrajectories.poses.clear();
 
    curPhantomData.X = 0;
    curPhantomData.Y = 0;
    curPhantomData.Z = 0;
    curPhantomData.ButtonState = 0;
    curPhantomData.Record = true;

    curGMMparams.nbStates = 6;		
    curGMMparams.nbVarPos = 2;		
    curGMMparams.nbDeriv = 3;		
    curGMMparams.dt = 0.0167;
    curGMMparams.nbData = 175;	  
    curGMMparams.nbSamples = 5;  	  
}

void BaxterMotionPrimitives::InitializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

	phantom_joint_sub = nh_.subscribe("/Phantom_joint_states", 1, &BaxterMotionPrimitives::PhantomJointCallback, this);

    // From the Unity
    unity_position_sub = nh_.subscribe("/MotionPrimitives/HandPoseStamped", 1, &BaxterMotionPrimitives::UnityRightHandControllerPositionCallback, this);
    unity_button_status_sub = nh_.subscribe("/MotionPrimitives/ButtonState", 1, &BaxterMotionPrimitives::UnityRightHandControllerButtonStatusCallback, this);

	phantom_button_sub = nh_.subscribe("/Phantom_button_state", 1, &BaxterMotionPrimitives::PhantomButtonCallback, this);
    sawyer_joint_sub = nh_.subscribe("/robot/joint_states", 1, &BaxterMotionPrimitives::RobotJointCallback, this);
    sawyer_endpoint_sub = nh_.subscribe("/robot/limb/right/endpoint_state", 1, &BaxterMotionPrimitives::RobotEndPointCallback, this);
}

void BaxterMotionPrimitives::InitializePublishers()
{
    ROS_INFO("Initializing Publishers");
    trajgmm_posArr_pub = nh_.advertise<geometry_msgs::PoseArray>("/pbdlib_trajgmm_states", 1);
}

void BaxterMotionPrimitives::InitializeServices()
{
    ROS_INFO("Initializing Services");
    //minimal_service_ = nh_.advertiseService("exampleMinimalService", &ExampleRosClass::serviceCallback, this); 
}

void BaxterMotionPrimitives::UnityRightHandControllerPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    curUnityData.X = msg->pose.position.x;
    curUnityData.Y = msg->pose.position.y;
    curUnityData.Z = msg->pose.position.z;

    std::cout << "[UNITY] Pose (X) : " << msg->pose.position.x << ", Pose (Y) : " << msg->pose.position.y << std::endl;
}

void BaxterMotionPrimitives::UnityRightHandControllerButtonStatusCallback(const std_msgs::Int32 &msg)
{
    /*RECORDING = 1,    // 1 (Button Dragging)
        JUST_STARTED, // 2 (Button Down)
        JUST_FINSHED  // 3 (Button Up)  */
    
    int status = msg.data;

    if (status == sVirualPoseRecStatus::RECORDING)
    {
        curUnityData.ButtonState = sVirualPoseRecStatus::RECORDING;
    }
    else if (status == sVirualPoseRecStatus::JUST_STARTED)
    {
        curUnityData.ButtonState = sVirualPoseRecStatus::JUST_STARTED;
    }
    else if (status == sVirualPoseRecStatus::JUST_FINSHED)
    {
        curUnityData.ButtonState = sVirualPoseRecStatus::JUST_FINSHED;
    }
}

void BaxterMotionPrimitives::PhantomJointCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    curPhantomData.X = msg->position.x;
    curPhantomData.Y = msg->position.y;
    curPhantomData.Z = msg->position.z;

    std::cout << "[PHANTOM] Pose (X) : " << msg->position.x << ", Pose (Y) : " << msg->position.y << std::endl;
}

void BaxterMotionPrimitives::PhantomButtonCallback(const std_msgs::String &msg)
{
    if (msg.data == "btn2drag")
    {
        curPhantomData.ButtonState = 1;
    }
    else if (msg.data == "btn2down")
    {
        curPhantomData.ButtonState = 2;
    }
    else if (msg.data == "btn2up")
    {
        curPhantomData.ButtonState = 3;
    }
    //std::cout << "Button State : " << nBtnState << std::endl;
}

void BaxterMotionPrimitives::RobotJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

}

void BaxterMotionPrimitives::RobotEndPointCallback(const baxter_core_msgs::EndpointState::ConstPtr &msg)
{
    sRobotEndPointPose   p = msg->pose;
    sRobotXYZCoord      pt = p.position;

    m_vRobotEndPt.position = pt;

   // std::cout << "Pose (X, Y, Z) : " << "("<< pt.x << "," << pt.y << "," << pt.z <<")" << std::endl;
}

void BaxterMotionPrimitives::StartDemonstration()
{
    ROS_INFO("Start Demonstration...");
    ROS_INFO("Move Phantom...");
    ROS_INFO("Push the Button while you are drawing something with the Phantom pen.");

    m_bIsTraining = true;
    m_nCurMotionCnt = 0;
}

void BaxterMotionPrimitives::TrainingPhantomData()
{
    if(curTrainDataType != RECORD_CURRENT_VIRTUAL_DATA)
        return;

    if(curGMMparams.nbSamples <= m_nCurMotionCnt){
        m_bIsTraining = false;
        m_nCurMotionCnt = -1;
    }

    m_bIsTraining = true;

    mat posData;

    if (curPhantomData.ButtonState == 1 || curPhantomData.ButtonState == 2)
    {
        m_vPhantomX.push_back((double)curPhantomData.X);
        m_vPhantomY.push_back((double)curPhantomData.Y);
        m_vPhantomZ.push_back((double)curPhantomData.Z);

        std::cout << "Save Phantom Data : (" << curPhantomData.X << "," << curPhantomData.Y << "," << curPhantomData.Z << ")" << std::endl;

        curPhantomData.Record = true;
    }
    if (curPhantomData.ButtonState == 3 && curPhantomData.Record)
    {
        if(m_bSaveFileFlag)
        {
            Row<double> rowX = rowvec(m_vPhantomX);
            Row<double> rowY = rowvec(m_vPhantomY);
            Row<double> rowZ = rowvec(m_vPhantomZ);

            posData.insert_rows(0, rowX);
            posData.insert_rows(1, rowY);
            posData.insert_rows(2, rowZ);

            posData.print();

            std::string csvPath = trainDataPath + std::to_string(m_nCurMotionCnt) + ".txt";
            diskio::save_csv_ascii(posData, csvPath);
        }
        else
        {
            m_mDemoTrainData.push_back(mat(posData));
        }
        
        m_vPhantomX.clear();
        m_vPhantomY.clear();
        m_vPhantomZ.clear();

        m_nCurMotionCnt++;

        curPhantomData.Record = false;
    }
}

void BaxterMotionPrimitives::TrainingUnityData()
{
    if(curTrainDataType != RECORD_CURRENT_VIRTUAL_DATA)
        return;

    if(curGMMparams.nbSamples <= m_nCurMotionCnt){
        m_bIsTraining = false;
        m_nCurMotionCnt = -1;
    }

    m_bIsTraining = true;

    mat posData;

    if (curUnityData.ButtonState == 1 || curUnityData.ButtonState == 2)
    {
        m_vUnityX.push_back((double)curUnityData.X);
        m_vUnityY.push_back((double)curUnityData.Y);
        m_vUnityZ.push_back((double)curUnityData.Z);

        std::cout << "Save Unity Data : (" << curUnityData.X << "," << curUnityData.Y << "," << curUnityData.Z << ")" << std::endl;

        curUnityData.Record = true;
    }
    if (curUnityData.ButtonState == 3 && curUnityData.Record)
    {
        if(m_bSaveFileFlag)
        {
            Row<double> rowX = rowvec(m_vUnityX);
            Row<double> rowY = rowvec(m_vUnityY);
            Row<double> rowZ = rowvec(m_vUnityZ);

            posData.insert_rows(0, rowX);
            posData.insert_rows(1, rowY);
            posData.insert_rows(2, rowZ);

            posData.print();

            std::string csvPath = trainDataPath + std::to_string(m_nCurMotionCnt) + ".txt";
            diskio::save_csv_ascii(posData, csvPath);
        }
        else
        {
            m_mDemoTrainData.push_back(mat(posData));
        }
        
        m_vUnityX.clear();
        m_vUnityY.clear();
        m_vUnityZ.clear();

        m_nCurMotionCnt++;

     3
    }3
}

void BaxterMotionPrimitives::MakeTrajectoryAlogrithm()
{
    int nbStates = curGMMparams.nbStates;		   
    int nbVarPos = curGMMparams.nbVarPos;		   
    int nbDeriv = curGMMparams.nbDeriv;		   
    double dt = curGMMparams.dt;         
    int nbData = curGMMparams.nbData;	       
    int nbSamples = curGMMparams.nbSamples;	       
    /**********************************************************************************************************/
    /*													Test TrajGMM learning from demos saved in txt files														*/
    /**********************************************************************************************************/
    // Demonstrations variables
    std::vector<Demonstration> demos;

    Demonstration demo = Demonstration(nbVarPos, nbData * nbSamples);
    std::vector<std::string> vars;

    vars.push_back("x1");
    vars.push_back("x2");
    vars.push_back("x3");
    vars.push_back("v1");
    vars.push_back("v2");
    vars.push_back("v3");
    vars.push_back("a1");
    vars.push_back("a2");
    vars.push_back("a3");

    pPhantomTrajGMM1 = NULL;
    
    pPhantomTrajGMM1 = new TrajGMM(nbStates, nbVarPos, nbDeriv, dt);
    pPhantomTrajGMM1->setVARSNames(vars);
    pPhantomTrajGMM1->constructPHI(nbData, nbSamples);

    std::string csvPath;

    // Files variables
    mat posData;
    std::string posDataPath;
    std::stringstream convert; // stringstream used for the conversion

    if(m_bUseTestTrainDataFlag && m_mDemoTrainData.size() == 0)
    {
        SetUseTestTrainDataFiles(1);

        // If we have trained data we already have, then use them
        bool bFileExists = true;
        for(int i=0;i<nbSamples;i++)
        {
            std::string filePath = trainDataPath + std::to_string(i) + ".txt";
            std::ifstream ifile(filePath);

            bFileExists &= (bool)ifile;
        }
        if(!bFileExists)
        {
            ROS_INFO("There is no trained data ....");
            ROS_INFO("Execute [1] Test Data (Phantom circle)....");

            if(pPhantomTrajGMM1 != NULL)
            {
                delete pPhantomTrajGMM1;
                pPhantomTrajGMM1 = NULL;
            }
            return;
        }
    }

    for(int k=1;k<= nbSamples;k++)
    {
        if(!m_bUseTestTrainDataFlag)
        {   
            posDataPath = trainDataPath + std::to_string(k - 1) + ".txt";
            posData.load(posDataPath);
        }else
        {
            posData = mat(m_mDemoTrainData[k]);
        }

        //std::cout << "size of posData : " << size(posData) << endl;

        int n_cols = posData.n_cols;
        //    int n_dataSize = 175;

        //std::cout << "n_cols : " << n_cols << endl;

        std::vector<double> t0, X0, Y0, Z0;

        for(int i=0;i<n_cols;i++)
        {
            double x = (double)i / (double) (n_cols-1);
            t0.push_back(x);
            X0.push_back(posData(0, i));
            Y0.push_back(posData(1, i));
            Z0.push_back(posData(2, i));
        }

        tk::spline s[3];
        s[0].set_points(t0,X0);    // currently it is required that X is already sorted
        s[1].set_points(t0,Y0);    // currently it is required that X is already sorted
        s[2].set_points(t0,Z0);    // currently it is required that X is already sorted

        vec t = linspace<vec>(0.0, 1, nbData);

        mat dataPts(3, nbData);

        for(int i=0;i<nbData;i++)
        {
            double x = (double) (t(i));                             // New t
            dataPts(0, i) = s[0](x);                                // New X
            dataPts(1, i) = s[1](x);                                // New Y
            dataPts(2, i) = s[2](x);                                // New Z
        }

        if(m_bSaveFileFlag)
        {
            csvPath = trainDataPath + "X_" + std::to_string(k-1) + ".txt";
            diskio::save_csv_ascii(dataPts, csvPath);
        }

        dataPts.reshape(nbVarPos * nbData, 1);
        mat trainingData = conv_to<mat>::from(pPhantomTrajGMM1->getPHI1()) * (dataPts * 1E2); //1E2 is used to avoid numerical computation problem
        trainingData.reshape(nbVarPos * pPhantomTrajGMM1->getNumDERIV(), nbData);

        // Storing transformed demo
        demo.getDatapoints().setData(trainingData);
        demo.getDatapoints().setVarNames(vars);
        demos.push_back(demo);
    }

    // Setting training data
    pPhantomTrajGMM1->setDEMONSTRATIONS(demos);

    // Learning and printing results
    cout << "\n Number of EM iterations: " << pPhantomTrajGMM1->EM_learn(1E-4);

    for (int i = 0; i < nbStates; i++)
    {
        cout << "\n Mu_" << i << " = \n"
                << pPhantomTrajGMM1->getMU(i);
        cout << "\n Sigma_" << i << " = \n"
                << pPhantomTrajGMM1->getSIGMA(i);
    }

    mat GAMMA = pPhantomTrajGMM1->getGamma();
    // 6 * 875 (MATLAB), here (875 * 6)

    cout << "size (sum(GAMMA,0))" << size(sum(GAMMA, 0)) << endl; // OK 1 X 6
    cout << "size (GAMMA) " << size(GAMMA) << endl;								// 6 * 875  (MATLAB), here (875 * 6)

    mat GAMMA2 = GAMMA / repmat(sum(GAMMA, 0), nbData * nbSamples, 1);

    cout << "size (GAMMA2) " << size(GAMMA2) << endl;

    // csvPath = basePath + "Test/RawPhantom/Debug/" + "GAMMA2.txt";
    // diskio::save_csv_ascii(GAMMA2, csvPath);

    uvec allcols = linspace<uvec>(0, nbStates - 1, nbStates);

    //cout << "\n GAMMA2 SIZE : " << size(GAMMA2) << " = \n" << endl;

    for (int i = 1; i <= 1 /*nbSamples*/; i++)
    {
        cout << "i :" << i << endl;
        uvec idTmp = linspace<uvec>((i - 1) * nbData, (i - 1) * nbData + nbData - 1, nbData);

        // idTmp.print("idTmp:");

        mat tempM = GAMMA2.submat(idTmp, allcols);
        cout << "size (tempM) " << size(tempM) << endl;

        ucolvec q = index_max(tempM, 1);
        cout << "size (colvec) " << size(q) << endl;

        // mat mat_q = conv_to<mat>::from(q);

        // csvPath = basePath + "Test/RawPhantom/Debug/" + "q" + std::to_string(i) + ".txt";
        // diskio::save_csv_ascii(mat_q, csvPath);

        //	cout << "\n tempM SIZE : " << size(tempM) << " = \n" << endl;
        //	cout << "\n q SIZE : " << size(q) << " = \n" << endl;

        mat optimalData = pPhantomTrajGMM1->leastSquaresOptimalData(arma::conv_to<colvec>::from(q)) * (1E-2);
        m_mDemoOptimalData = mat(optimalData);

        if(m_bSaveFileFlag)
        {            
            cout << "size (optimalData) " << size(optimalData) << endl;
            csvPath = trainDataPath + "optimalData1.txt";
            //csvPath = basePath + "Test/UMotion/Debug/" + "optimalData" + std::to_string(i) + ".txt";
            diskio::save_csv_ascii(optimalData, csvPath);
        }       
    }
    delete pPhantomTrajGMM1;
}
sRobotEndPointPose BaxterMotionPrimitives::GetCurRobotEndPos()
{
    ros::spinOnce();
    return m_vRobotEndPt;
}

sRobotEndPointPose BaxterMotionPrimitives::GetTargetRobotEndPos()
{
    return m_vRobotTargetEndPt;
}

sRobotEndPointPose BaxterMotionPrimitives::GetPrevRobotEndPos()
{
    return m_vRobotPrevPt;
}

sRobotEndPointTrajectory BaxterMotionPrimitives::GetPrevRobotEndTrajectories()
{
    return m_vRobotPrevTrajectories;
}

void BaxterMotionPrimitives::SendCmdMoveEndPointwithTrajectory()
{
    sRobotEndPointPose curRobotEndPt = GetCurRobotEndPos();
    
    mat Output;
    sRobotEndPointTrajectory msgPosArray;

    int nDataSize = curGMMparams.nbData;

    //std::string strOutputPath = basePath + "Test/UMotion/Debug/" + "optimalData" + std::to_string(s) + ".txt";
    
    if(!m_bUseTestTrainDataFlag || m_mDemoOptimalData.is_empty())
    { 
        std::string strOutputPath = trainDataPath + "optimalData1.txt";
        if(!Output.load(strOutputPath))
        {
            SetUseTestTrainDataFiles(1);
            ROS_INFO("There is no trained data ....");
            ROS_INFO("Execute [1] Test Data (Phantom circle)....");
            return;
        }
    }else
    {
        Output = m_mDemoOptimalData;
    }

    msgPosArray.poses.resize(nDataSize);

    sRobotXYZCoord      Txyz;

    Txyz.x = curRobotEndPt.position.x;
    Txyz.y = curRobotEndPt.position.y;
    Txyz.z = curRobotEndPt.position.z;

    m_vRobotPrevPt.position = Txyz;
    double dScale = 0.2;

    for (int j = 0; j < nDataSize; j++)
    {
        sRobotEndPointPose p;
        sRobotXYZCoord pt;
/*
        pt.z = (float)Output(0, j) / 100;
        pt.y = (float)Output(1, j) / 100 - 0.5;
        pt.x = (float)0.6;
*/
        pt.x = curRobotEndPt.position.x - (float)Output(0, j) * dScale;
        pt.y = curRobotEndPt.position.y - (float)Output(1, j) * dScale;
        pt.z = curRobotEndPt.position.z - (float)Output(2, j) * dScale;

        m_vRobotTargetEndPt.position = pt;
        m_vRobotPrevTrajectories.poses.push_back(m_vRobotTargetEndPt);

        p.position = pt;
        msgPosArray.poses[j] = p;        
    }
    trajgmm_posArr_pub.publish(msgPosArray);
    ROS_INFO("Publish !!....");
}

void BaxterMotionPrimitives::SendCmdMove(sRobotEndPointPose pose)
{
    sRobotEndPointPose curPose = GetTargetRobotEndPos();

    bool bArrived = false;
    ros::Time begin = ros::Time::now();

    double secs = ros::Time::now().toSec();
    ros::Duration d(5);
    secs = d.toSec();

    ros::Rate loop_rate(5);

    while (bArrived)
    {
        ros::Time end = ros::Time::now();
        ros::Duration elasped =  end - begin;
        if( elasped.toSec() > secs)
            bArrived = true;

        sRobotEndPointTrajectory msgPosArray;
        msgPosArray.poses.resize(1);
        msgPosArray.poses[0] = pose;

        vec error = zeros<vec>(3);
        //error[0] = deltaPos.x;
       // error[1] = deltaPos.y;
       // error[2] = deltaPos.z;
        
        double dDist = norm(error, 2);
        
        if( dDist < 0.001)
            bArrived = true;            

        trajgmm_posArr_pub.publish(msgPosArray);
        ros::spinOnce();
        loop_rate.sleep();   
    }

    /*
    int count = 0;


    sRobotEndPointTrajectory msgPosArrays;
    msgPosArrays.poses.resize(1);
    msgPosArrays.poses[0] = 0;


    GetCurRobotEndPos();

    while (msgPosArrays.poses[0] != pose) {

        
    sRobotEndPointTrajectory msgPosArray;
    msgPosArray.poses.resize(1);
    msgPosArray.poses[0] = pose;

    trajgmm_posArr_pub.publish(msgPosArray);
    ros::spinOnce();
    loop_rate.sleep();

    count++;
    }*/



/* 122   void get_current_joint_angles(double current_angles[7]){
 123     int i;
 124 
 125     //get a single message from the topic 'r_arm_controller/state'
 126     pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = 
 127       ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
 128       ("r_arm_controller/state");
 129     
 130     //extract the joint angles from it
 131     for(i=0; i<7; i++){
 132       current_angles[i] = state_msg->actual.positions[i];
 133     }
 134   }
 135 */

}

void BaxterMotionPrimitives::SendCmdMoveDemoPosition()
{
    sRobotEndPointPose pose;
    
    pose.position = StartPt;
    pose.orientation = Bottom;

    SendCmdMove(pose);
}

void BaxterMotionPrimitives::SetMaxRepeatMotions(int nMaxVal)
{
    m_nMaxMotionCnt = nMaxVal;
}

void BaxterMotionPrimitives::SetDataSize(int nDataSize)
{
    curGMMparams.nbData = nDataSize;
}

void BaxterMotionPrimitives::SetSaveFile(bool bSave)
{
    m_bSaveFileFlag = bSave;
}

void BaxterMotionPrimitives::SetUseTestTrainDataFiles(int nType)
{
    m_bUseTestTrainDataFlag = false;

    switch(nType)
    {
        case RECORD_CURRENT_VIRTUAL_DATA:
            m_bUseTestTrainDataFlag = true;
            trainDataPath = basePath + "Test/Debug/";
        break;

        case USE_TESTDATA_VIRTUAL_CIRCLE:
            trainDataPath = basePath + "Test/RawPhantom3/";
        break;

        case USE_TESTDATA_DRAW_U_MOTION:
            trainDataPath = basePath + "Test/UMotion/";
        break;

        case USE_TESTDATA_DRAW_V_MOTION:
            trainDataPath = basePath + "Test/VMotion/";
        break;

        default:
        break;
    }
    std::cout << "trainDataPath : " << trainDataPath << std::endl;
    curTrainDataType = nType;
}

void BaxterMotionPrimitives::SetGMMParams(sGMMParams params)
{
    curGMMparams.nbStates = params.nbStates;		        
    curGMMparams.nbVarPos = params.nbVarPos;		        
    curGMMparams.nbDeriv = params.nbDeriv;		            
    curGMMparams.dt = params.dt;   
    curGMMparams.nbData = params.nbData;
    curGMMparams.nbSamples = params.nbSamples;                   
}

void BaxterMotionPrimitives::SetGMMParams(int nbStates, int nbVarPos, int nbDeriv, double dt)
{
    curGMMparams.nbStates = nbStates;		        
    curGMMparams.nbVarPos = nbVarPos;		       
    curGMMparams.nbDeriv = nbDeriv;		         
    curGMMparams.dt = dt;                        
}

int BaxterMotionPrimitives::GetCurMotionCnt()
{
    return m_nCurMotionCnt;
}

sGMMParams BaxterMotionPrimitives::GetGMMParams()
{
    return curGMMparams;
}

bool BaxterMotionPrimitives::IsTraining()
{
    return m_bIsTraining;
}