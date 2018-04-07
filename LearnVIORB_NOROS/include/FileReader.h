//
// Created by zhuzunjie on 18-3-30.
//

#ifndef ROVIO_FILEREADER_H
#define ROVIO_FILEREADER_H

#include <atomic>
#include <string>
#include <fstream>
#include <Eigen/Dense>

#include "IMU/imudata.h"

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
void loadIMUFile(char * imuPath,std::vector<ORB_SLAM2::IMUData> &vimuData);
/*
 * @brief synchronize init time of imu and image.
 */
void synInit(std::vector<ICell> &imageList, std::vector<ORB_SLAM2::IMUData> &imuList, int imgIdx, int imuIdx);
/*
 * @brief read groundtruth information.
 * TODO: from now on we only read position, need to read orientation as well.
 */
void loadGTFile(const char * imuPath,std::vector<GT> &vGTData);



#endif //ROVIO_FILEREADER_H
