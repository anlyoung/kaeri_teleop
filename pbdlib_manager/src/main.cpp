#include "ros/ros.h"
#include <signal.h>
#include "MotionPrimitives.h"
#include "testCubic.h"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace pbdlib;

void mySigintHandler(int sig)
{
	ros::shutdown();
}

int main(int argc, char **argv)
{
	// Make ROS subscription
	ros::init(argc, argv, "pbdlibSimpleDemo");

	ros::NodeHandle nh;

	MotionPrimitives baxMP(&nh);
	signal(SIGINT, mySigintHandler);

	double fFramerate = 90.0;

	sGMMParams params;
	params.nbStates = 6; // Number of components in the GMM
	//params.nbVarPos = 2;		// Dimension of position data (here: x1,x2)
	params.nbVarPos = 3; // Dimension of position data (here: x1,x2,x3)

	params.nbDeriv = 3;   // Number of static&dynamic features (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
	params.dt = 1/fFramerate;   // Time step (without rescaling, large values such as 1 has the advantage of creating clusers based on position information)
	params.nbData = 175;  // Number of  datapoints in a trajectory
	params.nbSamples = 5; // Number of demonstrations

	baxMP.SetGMMParams(params);
	baxMP.SetUseTestTrainDataFiles(sTrainDataType::RECORD_CURRENT_VIRTUAL_DATA); // Start with trainDataPath = basePath + "Test/Current/"
	baxMP.SetDeviceType(sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER);

	ros::Rate rate(fFramerate);

	bool bQuit = false;

	ROS_INFO("Start Demonstrations (Pbdlib)");
	ROS_INFO("Press s and number of Demonstrations...(ex) s, 5");

	while (ros::ok())
	{
		std::string inputString;
		std::getline(std::cin, inputString);

		std::istringstream iss(inputString);
		std::string strCmd, strTemp;
		std::vector<std::string> strTokens;

		ros::spinOnce();

		while (getline(iss, strTemp, ','))
			strTokens.push_back(strTemp);

		strCmd = strTokens[0];

		if (strCmd.compare("s") == 0)
		{
			if (inputString.length() > 1)
			{
				params.nbSamples = std::stoi(strTokens[1]);
				baxMP.SetGMMParams(params);
			}

			baxMP.StartDemonstration();

			if (baxMP.GetVirtualDeviceType() == (int)sVirtualDeviceMode::MODE_PHANTOM_OMNI_DEVICE)
			{

				ROS_INFO("Start Demonstration...");
				ROS_INFO("Move Phantom...");
				ROS_INFO("Push the Button while you are drawing something with the Phantom pen.");
			}
			else if (baxMP.GetVirtualDeviceType() == (int)sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER)
			{
				ROS_INFO("Start Demonstration...");
				ROS_INFO("Move OCULUS Hand Controller...");
				ROS_INFO("Push the Button while you are drawing something with the controller.");
			}

			while (baxMP.IsTraining())
			{
				if (baxMP.GetVirtualDeviceType() == (int)sVirtualDeviceMode::MODE_PHANTOM_OMNI_DEVICE)
				{
					baxMP.TrainingPhantomData();
				}
				else if (baxMP.GetVirtualDeviceType() == (int)sVirtualDeviceMode::MODE_OCULUS_HAND_CONTROLLER)
				{
					baxMP.TrainingUnityData();
				}
				ros::spinOnce();
				rate.sleep();
			}
		}

		if (strCmd.compare("m") == 0)
		{
			std::cout << "Make Trajectory.." << std::endl;
			baxMP.MakeTrajectoryAlogrithm();
			std::cout << "Data Cleared : " << baxMP.GetCurMotionCnt() << std::endl;
		}
		else if (strCmd.compare("g") == 0)
		{
			baxMP.PrintOptimalData();
		}
		else if (strCmd.compare("q") == 0)
		{
			bQuit = true;
			ROS_INFO("QUIT Demonstrations....");
			break;
		}
		rate.sleep();
	}

	return 0;
}
// %EndTag(FULLTEXT)%
