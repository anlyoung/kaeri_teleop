//https://android.googlesource.com/platform/external/eigen/+/jb-mr1.1-dev/unsupported/test/splines.cpp
#include "testCubic.h"
#include "spline.h"
#include <fstream>

#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace std;

void test_splineSimple()
{
   std::vector<double> X(5), Y(5);
   X[0]=0.1; X[1]=0.4; X[2]=1.2; X[3]=1.8; X[4]=2.0;
   Y[0]=0.1; Y[1]=0.7; Y[2]=0.6; Y[3]=1.1; Y[4]=0.9;

   tk::spline s;
   s.set_points(X,Y);    // currently it is required that X is already sorted

   double x=1.5;

   printf("spline at %f is %f\n", x, s(x));
}

void test_splineDetailed()
{
  // Files variables
  mat posData;
  std::string posDataPath;
  std::stringstream convert; // stringstream used for the conversion
  std::string csvPath;

  for(int k=1;k<=5;k++)
  {
    std::string basePath = "/home/user/module_ws/src/pbdlib_manager/";
    posDataPath = basePath + "Test/RawPhantom2/" + std::to_string(k - 1) + ".txt";
    cout << posDataPath << endl;
    posData.load(posDataPath);

    //std::cout << "size of posData : " << size(posData) << endl;

    int n_cols = posData.n_cols;
    int n_dataSize = 175;

    std::cout << "n_cols : " << n_cols << endl;

    std::vector<double> t0, X0, Y0, Z0;

    for(int i=0;i<n_cols;i++)
    {
      double x = (double)i / (double) (n_cols - 1);
      t0.push_back(x);
      X0.push_back(posData(0, i));
      Y0.push_back(posData(1, i));
      Z0.push_back(0);
    }

    tk::spline s[3];
    s[0].set_points(t0,X0);    // currently it is required that X is already sorted
    s[1].set_points(t0,Y0);    // currently it is required that X is already sorted
    s[2].set_points(t0,Z0);    // currently it is required that X is already sorted

    vec t = linspace<vec>(0.0, 1.0, n_dataSize);

    mat dataPts(3, n_dataSize);

    for(int i=0;i<n_dataSize;i++)
    {
      double x = (double) (t(i));      // New t
      dataPts(0, i) = s[0](x);         // New X
      dataPts(1, i) = s[1](x);         // New Y
      dataPts(2, i) = s[2](x);         // New Z
    }
    csvPath = basePath + "Test/RawPhantom2/Debug/" + "X_" + std::to_string(k-1) + ".txt";
    diskio::save_csv_ascii(dataPts, csvPath);
  }
}