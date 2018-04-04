#pragma once

#include <atomic>
#include <string>
#include <fstream>

#include <Eigen/Dense>
#include <src/IMU/imudata.h>

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
/*
 * @brief synchronize init time of imu and image.
 */
void initData(std::vector<ICell> &imageList, std::vector<ORB_SLAM2::IMUData> &imuList, int imgIdx, int imuIdx);
/*
 * @brief read groundtruth information.
 * TODO: from now on we only read position, need to read orientation as well.
 */
void loadGTFile(const char * imuPath,std::vector<GT> &vGTData);

