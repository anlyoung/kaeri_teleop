#include "MotionPrimitives.h"
#include "spline.h"

// https://github.com/ros-industrial/cros/blob/master/resources/cros_testbed/src/std_msgs_talker.cpp

MotionPrimitives::MotionPrimitives(ros::NodeHandle* nodehandle)
    :nh_(*nodehandle)
{
    // constructor
    ROS_INFO("in class constructor of MotionPrimitives");

    InitializeSubscribers();
    InitializePublishers();
    InitializeServices();

    m_nCurMotionCnt = 0;
    m_nMaxMotionCnt = 0;
    m_bIsTraining = false;
    m_bSaveFileFlag = true;
    m_bUseTestTrainDataFlag = false;

    basePath =  ros::package::getPath("pbdlib_manager") + "/";
		std::cout << "basePath:" << basePath << std::endl;
    trainDataPath = basePath + "Test/Unity/";
    curTrainDataType = sTrainDataType::RECORD_CURRENT_VIRTUAL_DATA;
    curVirtualDeviceType = sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER;
 
    curPhantomData.X = 0;
    curPhantomData.Y = 0;
    curPhantomData.Z = 0;
    curPhantomData.ButtonState = 0;
    curPhantomData.Record = false;

    curUnityData.X = 0;
    curUnityData.Y = 0;
    curUnityData.Z = 0;
    curUnityData.qX = 0;
    curUnityData.qY = 0;
    curUnityData.qZ = 0;
    curUnityData.qW = 0;
    curUnityData.ButtonState = 0;
    curUnityData.Record = false;

    curGMMparams.nbStates = 6;		
    curGMMparams.nbVarPos = 2;		
    curGMMparams.nbDeriv = 3;		
    curGMMparams.dt = 0.0167;
    curGMMparams.nbData = 175;	  
    curGMMparams.nbSamples = 5;  	  
}

void MotionPrimitives::SetUseTestTrainDataFiles(int nType)
{
    m_bUseTestTrainDataFlag = false;

    switch(nType)
    {
        case sTrainDataType::RECORD_CURRENT_VIRTUAL_DATA:
            m_bUseTestTrainDataFlag = true;

            if(curVirtualDeviceType == sVirtualDeviceMode::MODE_PHANTOM_OMNI_DEVICE)
                trainDataPath = basePath + "Test/Phantom/";
            else if(curVirtualDeviceType == sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER)
                trainDataPath = basePath + "Test/Unity/";
        break;

        case sTrainDataType::USE_TESTDATA_VIRTUAL_CIRCLE:
            trainDataPath = basePath + "Test/RawPhantom3/";
        break;

        case sTrainDataType::USE_TESTDATA_DRAW_U_MOTION:
            trainDataPath = basePath + "Test/UMotion/";
        break;

        case sTrainDataType::USE_TESTDATA_DRAW_V_MOTION:
            trainDataPath = basePath + "Test/VMotion/";
        break;

        default:
        break;
    }
    //std::cout << "trainDataPath : " << trainDataPath << std::endl;
    curTrainDataType = (sTrainDataType)nType;
}

void MotionPrimitives::SetDeviceType(int nType)
{
    /* 
    MODE_PHANTOM_OMNI_DEVICE,
    MODE_OCULUS_HAND_CONTROLLER,        // Requre 6 joints
    */
    switch(nType)
    {
        case sVirtualDeviceMode::MODE_PHANTOM_OMNI_DEVICE:
        break;
        case sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER:
        break;
        default:
        break;
    }

    curVirtualDeviceType = (sVirtualDeviceMode)nType;    
}

void MotionPrimitives::InitializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");

	phantom_joint_sub = nh_.subscribe("/Phantom_joint_states", 1, &MotionPrimitives::PhantomJointCallback, this);

    // From the Unity
    unity_position_sub = nh_.subscribe("/MotionPrimitives/HandPoseStamped", 1, &MotionPrimitives::UnityRightHandControllerPositionCallback, this);
    unity_button_status_sub = nh_.subscribe("/MotionPrimitives/ButtonState", 1, &MotionPrimitives::UnityRightHandControllerButtonStatusCallback, this);
	phantom_button_sub = nh_.subscribe("/Phantom_button_state", 1, &MotionPrimitives::PhantomButtonCallback, this);
}

void MotionPrimitives::InitializePublishers()
{
    ROS_INFO("Initializing Publishers");
}

void MotionPrimitives::InitializeServices()
{
    ROS_INFO("Initializing Services");
    
    //motion_primitives_serv = nh_.advertiseService("/MotionPrimitives/OptimalData", &BaxterMotionPrimitives::serviceCallback, this); 
}
/*
//member function implementation for a service callback function
bool BaxterMotionPrimitives::serviceCallback(example_srv::simple_bool_service_messageRequest& request, example_srv::simple_bool_service_messageResponse& response) {
    ROS_INFO("service callback activated");
    response.resp = true; // boring, but valid response info
    return true;
}
 */

void MotionPrimitives::UnityRightHandControllerPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    curUnityData.X = msg->pose.position.x;
    curUnityData.Y = msg->pose.position.y;
    curUnityData.Z = msg->pose.position.z;

    curUnityData.qX = msg->pose.orientation.x;
    curUnityData.qY = msg->pose.orientation.y;
    curUnityData.qZ = msg->pose.orientation.z;
    curUnityData.qW = msg->pose.orientation.z;    

   // std::cout << "[UNITY] Pose (X) : " << msg->pose.position.x << ", Pose (Y) : " << msg->pose.position.y << std::endl;
}

void MotionPrimitives::UnityRightHandControllerButtonStatusCallback(const std_msgs::Int32 &msg)
{
    int status = msg.data;
    curUnityData.ButtonState = status;
}

void MotionPrimitives::PhantomJointCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    curPhantomData.X = msg->position.x;
    curPhantomData.Y = msg->position.y;
    curPhantomData.Z = msg->position.z;

    // Change orientations to Quaternion
    std::cout << "[PHANTOM] Pose (X) : " << msg->position.x << ", Pose (Y) : " << msg->position.y << std::endl;
}

void MotionPrimitives::PhantomButtonCallback(const std_msgs::String &msg)
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

void MotionPrimitives::StartDemonstration()
{
    m_bIsTraining = true;
    m_nCurMotionCnt = 0;
}

bool MotionPrimitives::TrainingPhantomData()
{
    if(curTrainDataType != sTrainDataType::RECORD_CURRENT_VIRTUAL_DATA)
    {
        m_bIsTraining = false;
        return false;
    }

    if (curVirtualDeviceType != sVirtualDeviceMode::MODE_PHANTOM_OMNI_DEVICE)
    {
        m_bIsTraining = false;
        return false;
    }

    if(curGMMparams.nbSamples <= m_nCurMotionCnt){
        m_bIsTraining = false;
        return false;
    }

    m_bIsTraining = true;

    mat posData;

    curPhantomData.Record = false;

    if(curPhantomData.ButtonState == (int)sVirtualPoseRecStatus::JUST_STARTED)
    {
        curPhantomData.Record = true;
    }

    if((curPhantomData.Record == true) && (curPhantomData.ButtonState == (int)sVirtualPoseRecStatus::RECORDING))
    {
        m_vPhantomX.push_back((double)curPhantomData.X);
        m_vPhantomY.push_back((double)curPhantomData.Y);
        m_vPhantomZ.push_back((double)curPhantomData.Z);

        std::cout << "Save Phantom Data : (" << curPhantomData.X << "," << curPhantomData.Y << "," << curPhantomData.Z << ")" << std::endl;
    }

    if(curPhantomData.ButtonState == (int)sVirtualPoseRecStatus::JUST_FINSHED)
    {
        curPhantomData.Record = false;
    }

    if((curPhantomData.Record == false) && (curPhantomData.ButtonState == (int)sVirtualPoseRecStatus::JUST_FINSHED))
    {
        if(m_bSaveFileFlag)
        {
            Row<double> rowX = rowvec(m_vPhantomX);
            Row<double> rowY = rowvec(m_vPhantomY);
            Row<double> rowZ = rowvec(m_vPhantomZ);

            posData.insert_rows(0, rowX);
            posData.insert_rows(1, rowY);
            posData.insert_rows(2, rowZ);

            //posData.print();

            std::string csvPath = trainDataPath + std::to_string(m_nCurMotionCnt) + ".txt";
            diskio::save_csv_ascii(posData, csvPath);

            m_nCurMotionCnt++;
        }
        else
        {
            m_mDemoTrainData.push_back(mat(posData));
        }
        
        m_vPhantomX.clear();
        m_vPhantomY.clear();
        m_vPhantomZ.clear();


        curPhantomData.Record = false;
    }
    return true;
}

bool MotionPrimitives::TrainingUnityData()
{
    if (curTrainDataType != sTrainDataType::RECORD_CURRENT_VIRTUAL_DATA)
    {
        m_bIsTraining = false;
        return false;
    }

    if (curVirtualDeviceType != sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER)
    {
        m_bIsTraining = false;
        return false;
    }

    if (m_nCurMotionCnt > curGMMparams.nbSamples)
    {
        m_bIsTraining = false;
        return false;
    }

    mat posData;

    curUnityData.Record = false;

    if(curUnityData.ButtonState == (int)sVirtualPoseRecStatus::JUST_STARTED)
    {
        curUnityData.Record = true;
    }

    else if(curUnityData.ButtonState == (int)sVirtualPoseRecStatus::RECORDING)
    {
        m_vUnityX.push_back((double)curUnityData.X);
        m_vUnityY.push_back((double)curUnityData.Y);
        m_vUnityZ.push_back((double)curUnityData.Z);

        std::cout << "Save Unity Data : (" << curUnityData.X << "," << curUnityData.Y << "," << curUnityData.Z << ")" << std::endl;
    }

    else if(curUnityData.ButtonState == (int)sVirtualPoseRecStatus::JUST_FINSHED)
    {
        curUnityData.Record = false;
     
        if(m_bSaveFileFlag)
        {
            Row<double> rowX = rowvec(m_vUnityX);
            Row<double> rowY = rowvec(m_vUnityY);
            Row<double> rowZ = rowvec(m_vUnityZ);

            posData.insert_rows(0, rowX);
            posData.insert_rows(1, rowY);
            posData.insert_rows(2, rowZ);

						if(posData.size() > 10)
						{
            	std::string csvPath = trainDataPath + std::to_string(m_nCurMotionCnt) + ".txt";
            	diskio::save_csv_ascii(posData, csvPath);
            	m_nCurMotionCnt++;
						}
        }
        else
        {
            m_mDemoTrainData.push_back(mat(posData));
        }

        m_vUnityX.clear();
        m_vUnityY.clear();
        m_vUnityZ.clear();
        
    }

    if(m_nCurMotionCnt == curGMMparams.nbSamples){
				m_nCurMotionCnt = 0;
        m_bIsTraining = false;
    }

    return true;
}

int MotionPrimitives::GetVirtualDeviceType()
{
		//std::cout << curVirtualDeviceType << std::endl;
    return (int)curVirtualDeviceType;
}

bool MotionPrimitives::MakeTrajectoryAlogrithm()
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

    TrajGMM *pVirtualTrajGMM1 = new TrajGMM(nbStates, nbVarPos, nbDeriv, dt);
    pVirtualTrajGMM1->setVARSNames(vars);
    pVirtualTrajGMM1->constructPHI(nbData, nbSamples);

    std::string csvPath;

    // Files variables
    mat posData;
    std::string posDataPath;
    std::stringstream convert; // stringstream used for the conversion

    if(m_bUseTestTrainDataFlag && m_mDemoTrainData.size() == 0)
    {
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
            SetUseTestTrainDataFiles(1);

            if(pVirtualTrajGMM1 != NULL)
            {
                delete pVirtualTrajGMM1;
                pVirtualTrajGMM1 = NULL;
            }
						SetUseTestTrainDataFiles(1);
            return false;
        }
    }    

    for(int k=1;k<= nbSamples;k++)
    {
        if(m_mDemoTrainData.size() == 0)
        {   
            posDataPath = trainDataPath + std::to_string(k - 1) + ".txt";
            posData.load(posDataPath);
        }else
        {
            posData = mat(m_mDemoTrainData[k-1]);
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
        mat trainingData = conv_to<mat>::from(pVirtualTrajGMM1->getPHI1()) * (dataPts * 1E2); //1E2 is used to avoid numerical computation problem
        trainingData.reshape(nbVarPos * pVirtualTrajGMM1->getNumDERIV(), nbData);

        // Storing transformed demo
        demo.getDatapoints().setData(trainingData);
        demo.getDatapoints().setVarNames(vars);
        demos.push_back(demo);
    }

    // Setting training data
    pVirtualTrajGMM1->setDEMONSTRATIONS(demos);

    // Learning and printing results
    cout << "\n Number of EM iterations: " << pVirtualTrajGMM1->EM_learn(1E-4);

    for (int i = 0; i < nbStates; i++)
    {
        cout << "\n Mu_" << i << " = \n"
                << pVirtualTrajGMM1->getMU(i);
        cout << "\n Sigma_" << i << " = \n"
                << pVirtualTrajGMM1->getSIGMA(i);
    }

    mat GAMMA = pVirtualTrajGMM1->getGamma();
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

        mat optimalData = pVirtualTrajGMM1->leastSquaresOptimalData(arma::conv_to<colvec>::from(q)) * (1E-2);
        m_mDemoOptimalData = mat(optimalData);

        if(m_bSaveFileFlag)
        {            
            cout << "size (optimalData) " << size(optimalData) << endl;
            csvPath = trainDataPath + "optimalData1.txt";
            //csvPath = basePath + "Test/UMotion/Debug/" + "optimalData" + std::to_string(i) + ".txt";
            diskio::save_csv_ascii(optimalData, csvPath);
        }       
    }
    delete pVirtualTrajGMM1;

    return true;
}

bool MotionPrimitives::GetOptimalTrajectory(mat &OptimalMatData)
{
    sRobotEndPointTrajectory msgPosArray;

    int nDataSize = curGMMparams.nbData;

    //std::string strOutputPath = basePath + "Test/UMotion/Debug/" + "optimalData" + std::to_string(s) + ".txt";
    
    if(!m_bUseTestTrainDataFlag || m_mDemoOptimalData.is_empty())
    { 
        std::string strOutputPath = trainDataPath + "optimalData1.txt";
        if(!OptimalMatData.load(strOutputPath))
        {
            SetUseTestTrainDataFiles(1);
            ROS_INFO("There is no trained data ....");
            ROS_INFO("Execute [1] Test Data (Phantom circle)....");
            return false;
        }
    }else
    {
        OptimalMatData = mat(m_mDemoOptimalData);
    }
    return true;
  /* msgPosArray.poses.resize(nDataSize);

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

        pt.x = curRobotEndPt.position.x - (float)Output(0, j) * dScale;
        pt.y = curRobotEndPt.position.y - (float)Output(1, j) * dScale;
        pt.z = curRobotEndPt.position.z - (float)Output(2, j) * dScale;

        m_vRobotTargetEndPt.position = pt;
        m_vRobotPrevTrajectories.poses.push_back(m_vRobotTargetEndPt);

        p.position = pt;
        msgPosArray.poses[j] = p;        
    }
    trajgmm_posArr_pub.publish(msgPosArray);
    
    ROS_INFO("Publish !!...."); */  
}

void MotionPrimitives::PrintOptimalData()
{
    mat OptimalMatData;
    bool bRes = GetOptimalTrajectory(OptimalMatData);

    if(bRes)
    {
        std::cout << "size of OptimalData : " << size(OptimalMatData) << endl;
        std::cout << "Data : " << OptimalMatData << endl;
    }
}

void MotionPrimitives::SetMaxRepeatMotions(int nMaxVal)
{
    m_nMaxMotionCnt = nMaxVal;
}

void MotionPrimitives::SetDataSize(int nDataSize)
{
    curGMMparams.nbData = nDataSize;
}

void MotionPrimitives::SetSaveFile(bool bSave)
{
    m_bSaveFileFlag = bSave;
}

void MotionPrimitives::SetGMMParams(sGMMParams params)
{
    curGMMparams.nbStates = params.nbStates;		        
    curGMMparams.nbVarPos = params.nbVarPos;		        
    curGMMparams.nbDeriv = params.nbDeriv;		            
    curGMMparams.dt = params.dt;   
    curGMMparams.nbData = params.nbData;
    curGMMparams.nbSamples = params.nbSamples;                   
}

void MotionPrimitives::SetGMMParams(int nbStates, int nbVarPos, int nbDeriv, double dt)
{
    curGMMparams.nbStates = nbStates;		        
    curGMMparams.nbVarPos = nbVarPos;		       
    curGMMparams.nbDeriv = nbDeriv;		         
    curGMMparams.dt = dt;                        
}

int MotionPrimitives::GetCurMotionCnt()
{
    return m_nCurMotionCnt;
}

sGMMParams MotionPrimitives::GetGMMParams()
{
    return curGMMparams;
}

bool MotionPrimitives::IsTraining()
{
    return m_bIsTraining;
}
