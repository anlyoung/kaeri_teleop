#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"

#include "math.h"
#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>

#include "pbdlib/trajgmm.h"
#include "pbdlib/datapoints.h"
#include "pbdlib/demonstration.h"
#include "pbdlib/gmm.h"
#include "pbdlib/gmr.h"
#include <sstream>

#include <armadillo>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace pbdlib;
using namespace Eigen;

std::string basePath = "/home/user/module_ws/src/pbdlib_manager/";

float fPhantomX = 0, fPhantomY = 0;
int nBtnState = 0;
bool is_drawing_demonstration = false;
bool must_recompute_GMR = false;

int index_demostration = 0;

// Trajectory model initialization
TrajGMM *pPhantomTrajGMM1;

std::vector<double> vPhantomX;
std::vector<double> vPhantomY;

Eigen::MatrixXd readCSV(std::string file, int rows, int cols)
{

	std::ifstream in(file);

	std::string line;

	int row, col = 0;

	Eigen::MatrixXd res = Eigen::MatrixXd(rows, cols);

	if (in.is_open())
	{

		while (std::getline(in, line))
		{

			char *ptr = (char *)line.c_str();
			int len = line.length();

			col = 0;

			char *start = ptr;
			for (int i = 0; i < len; i++)
			{

				if (ptr[i] == ',')
				{
					res(row, col++) = atof(start);
					start = ptr + i + 1;
				}
			}
			res(row, col) = atof(start);

			row++;
		}

		in.close();
	}
	return res;
}

void PhantomJointCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
	//fPhantomX = -(msg->position.y - 1.5) * 2.0;
	//fPhantomY = -(msg->position.z - 1.5) * 3.0;

	fPhantomX = msg->position.y; //-(msg->position.y - 1.5) * 2.0;
	fPhantomY = msg->position.z; //-(msg->position.z - 1.5) * 3.0;
	//std::cout << "Pose (X) : " << msg->position.x << ", Pose (Y) : " << msg->position.y << std::endl;
}

void PhantomButtonCallback(const std_msgs::String &msg)
{
	if (msg.data == "btn2drag")
	{
		nBtnState = 1;
	}
	else if (msg.data == "btn2down")
	{
		nBtnState = 2;
	}
	else if (msg.data == "btn2up")
	{
		nBtnState = 3;
	}

	std::cout << "Button State : " << nBtnState << std::endl;
}

bool bRecord = true;
int iGlobalCnt = 0;

void TrainingPhantomData()
{
	mat posData;

	if (nBtnState == 1 || nBtnState == 2)
	{
		vPhantomX.push_back((double)fPhantomX);
		vPhantomY.push_back((double)fPhantomY);

		std::cout << "Save Phantom Data : (" << fPhantomX << "," << fPhantomY << ")" << std::endl;

		bRecord = true;
	}
	if (nBtnState == 3 && bRecord)
	{
		Row<double> rowX = rowvec(vPhantomX);
		Row<double> rowY = rowvec(vPhantomY);

		posData.insert_rows(0, rowX);
		posData.insert_rows(1, rowY);

		posData.print();

		std::string csvPath = basePath + "Test/RawPhantom/" + std::to_string(index_demostration) + ".txt";
		diskio::save_csv_ascii(posData, csvPath);

		vPhantomX.clear();
		vPhantomY.clear();
		index_demostration++;

		iGlobalCnt++;

		bRecord = false;
	}
}

void MakeTrajectoryAlogrithm()
{
	int nbStates = 6;		// Number of components in the GMM
	int nbVarPos = 2;		// Dimension of position data (here: x1,x2)
	int nbDeriv = 3;		// Number of static&dynamic features (D=2 for [x,dx], D=3 for [x,dx,ddx], etc.)
	double dt = 0.0167; // Time step (without rescaling, large values such as 1 has the advantage of creating clusers based on position information)
	int nbData = 175;		// Number of  datapoints in a trajectory
	int nbSamples = 5;	// Number of demonstrations

	/**********************************************************************************************************/
	/*													Test TrajGMM learning from demos saved in txt files														*/
	/**********************************************************************************************************/
	// Demonstrations variables
	std::vector<Demonstration> demos;
	Demonstration demo = Demonstration(nbVarPos, nbData * nbSamples);
	std::vector<std::string> vars;

	vars.push_back("x1");
	vars.push_back("x2");
	vars.push_back("v1");
	vars.push_back("v2");
	vars.push_back("a1");
	vars.push_back("a2");

	pPhantomTrajGMM1 = new TrajGMM(nbStates, nbVarPos, nbDeriv, dt);
	pPhantomTrajGMM1->setVARSNames(vars);
	pPhantomTrajGMM1->constructPHI(nbData, nbSamples);

	std::string csvPath;

	// Files variables
	mat posData;
	std::string posDataPath;
	std::stringstream convert; // stringstream used for the conversion

	// Loading and storing demos
	for (int i = 1; i <= nbSamples; i++)
	{
		posDataPath = basePath + "Test/UMotion/" + std::to_string(i - 1) + ".txt";
		//		posDataPath = basePath + "Test/RawPhantom/" + std::to_string(i-1) + ".txt";
		cout << posDataPath << endl;

		// mat tmpPosData;
		// tmpPosData.load(posDataPath);
		// uvec idTmp = linspace<uvec>(0, tmpPosData., nbData);

		posData.load(posDataPath);

		posData.reshape(nbVarPos * nbData, 1);
		mat trainingData = conv_to<mat>::from(pPhantomTrajGMM1->getPHI1()) * (posData * 1E2); //1E2 is used to avoid numerical computation problem
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

		// csvPath = basePath + "Test/RawPhantom/Debug/" + "Mu_" + std::to_string(i) + ".txt";
		// diskio::save_csv_ascii(pPhantomTrajGMM1->getMU(i), csvPath);

		// csvPath = basePath + "Test/RawPhantom/Debug/" + "Sigma_" + std::to_string(i) + ".txt";
		// diskio::save_csv_ascii(pPhantomTrajGMM1->getSIGMA(i), csvPath);
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

		mat optimalData = pPhantomTrajGMM1->leastSquaresOptimalData(arma::conv_to<colvec>::from(q));

		cout << "size (optimalData) " << size(optimalData) << endl;
		//csvPath = basePath + "Test/RawPhantom/Debug/" + "optimalData" + std::to_string(i) + ".txt";
		csvPath = basePath + "Test/UMotion/Debug/" + "optimalData" + std::to_string(i) + ".txt";
		diskio::save_csv_ascii(optimalData, csvPath);

		//mat optimalData2 = join_cols(optimalData.cols, tmpMu.cols );
	}
	delete pPhantomTrajGMM1;
}

ros::Publisher pub_trajgmm;

void SendData()
{
	mat Output;
	geometry_msgs::PoseArray msgPosArray;

	int nDataSize = 0;

	for (int s = 1; s < 2; s++)
	{
		std::string strOutputPath = basePath + "Test/UMotion/Debug/" + "optimalData" + std::to_string(s) + ".txt";
		//std::string strOutputPath = basePath + "Test/RawPhantom/Debug/" + "optimalData" + std::to_string(s) + ".txt";
		Output.load(strOutputPath);

		nDataSize = 175; //Output.();
		msgPosArray.poses.resize(nDataSize);

		for (int j = 0; j < nDataSize; j++)
		{
			geometry_msgs::Pose p;
			geometry_msgs::Point pt;

			//			pt.x = (float)Output(0, j) /100;
			//			pt.y = (float)Output(1, j) /100;

			pt.z = (float)Output(0, j) / 100;
			pt.y = (float)Output(1, j) / 100;
			pt.x = (float)0.5;

			p.position = pt;
			msgPosArray.poses[j] = p;
		}
	}
	pub_trajgmm.publish(msgPosArray);
}

int main(int argc, char **argv)
{
	// Make ROS subscription
	ros::init(argc, argv, "PhantomListener");

	ros::NodeHandle n;
	ros::Subscriber phantom_joint_sub = n.subscribe("/Phantom_joint_states", 1, PhantomJointCallback);
	ros::Subscriber phantom_button_pub = n.subscribe("/Phantom_button_state", 1, PhantomButtonCallback);

	pub_trajgmm = n.advertise<geometry_msgs::PoseArray>("/pbdlib_trajgmm_states", 1);
	ros::Rate rate(60);

	while (ros::ok())
	{
		std::string inputString;
		std::getline(std::cin, inputString);

		if (inputString.compare("S") == 0)
		{
			std::cout << "Start Demonstration..." << std::endl;
			std::cout << "Move Phantom..." << std::endl;
			std::cout << "Push the Button while you are drawing with the Phantom pen." << std::endl;

			while (iGlobalCnt < 5)
			{
				TrainingPhantomData();
				ros::spinOnce();
				rate.sleep();
			}
		}
		else if (inputString.compare("m") == 0)
		{
			std::cout << "Make Trajectory.." << std::endl;

			MakeTrajectoryAlogrithm();
			//test_spline_3();

			std::cout << "Data Cleared : " << index_demostration << std::endl;

			//PublishToTheRobot;
		}
		else if (inputString.compare("r") == 0)
		{
			SendData();
			ros::spinOnce();
		}
	}
	//func1();

	//func2();

	return 0;
}
// %EndTag(FULLTEXT)%
