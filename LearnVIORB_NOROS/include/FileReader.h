#pragma once

#include <atomic>
#include <string>
#include <fstream>
#include "../src/IMU/imudata.h"
#include <Eigen/Dense>

using namespace std;

typedef struct ImageList
{
	double timeStamp;
	string imgName;
}ICell;

typedef struct loadGroundtruth
{
	double timeStamp;
	Eigen::Vector3d position;
	Eigen::Quaterniond rotation_Q;
	
	
}GT;

void loadImageList(char * imagePath,std::vector<ICell> &iListData);
void loadBlurImageList(char * imagePath,std::vector<ICell> &iListData);
void loadIMUFile(char * imuPath,std::vector<ORB_SLAM2::IMUData> &vimuData);
void loadBlurIMUFile(char * imuPath,std::vector<ORB_SLAM2::IMUData> &vimuData);
void loadGTFile(const char * imuPath,std::vector<GT> &vGTData);

