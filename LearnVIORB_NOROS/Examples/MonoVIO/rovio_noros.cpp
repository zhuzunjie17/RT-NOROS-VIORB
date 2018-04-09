//
// Created by zhuzunjie on 18-4-1.
//

#include <iostream>
#include <iomanip>

#include "opencv2/core/core.hpp"
#include "glog/logging.h"

#include "FileReader.h"
#include "IMU/imudata.h"
//#include "IMU/configparam.h"

#include "rovio/RovioNode.hpp"
#include "rovio/RovioFilter.hpp"

static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
static constexpr int nCam_ = 1; // Used total number of cameras.
static constexpr int nPose_ = 0; // Additional pose states.

typedef rovio::RovioFilter<rovio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;
using namespace std;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: "
                "path_to_cam0/data.csv "
                "path_to_cam0/data"
                "path_to_imu/data.csv" << endl;
        return -1;
    }
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "../logs";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    vector<ICell> vImageList;
    loadImageList(argv[1], vImageList);
    vector<ORB_SLAM2::IMUData> vIMUList;
    loadIMUFile(argv[3],vIMUList);

    // 预处理数据,滤波器确实不需要初始同步，但这里的初始同步还是先放着。
    int imageIdx=0;
    int imuIdx=0;
    synInit(vImageList,vIMUList,imageIdx,imuIdx);

    vector<double> vTimeCost;///<记录每帧时间

    double e = pow(10.0, -9);
    string imdir = argv[2];
    string filter_config = "/home/zhuzunjie/Projects/RT-NOROS-VIORB/LearnVIORB_NOROS/config/rovio.info";

    std::shared_ptr<mtFilter> mpFilter(new mtFilter);
    mpFilter->readFromInfo(filter_config);
    std::string camera_config = "/home/zhuzunjie/Projects/RT-NOROS-VIORB/LearnVIORB_NOROS/config/euroc_cam0.yaml";
    mpFilter->cameraCalibrationFile_[0] = camera_config;
    mpFilter->refreshProperties();
    rovio::RovioNode<mtFilter> rovioNode(mpFilter);
    rovioNode.makeTest();

    for (int id = 0; id < imuIdx; ++id) {
        rovioNode.imuCallback(vIMUList[id],vIMUList[id]._t*e);
    }
    for (; imageIdx < vImageList.size(); ++imageIdx)
    {
        //load image
        LOG(INFO) << "<<<<<<<<<<<<<LOAD IMAGE " << imageIdx << " : " << vImageList[imageIdx].imgName;
        string impath = imdir+"/"+vImageList[imageIdx].imgName;
        cv::Mat im = cv::imread(impath,0);
        LOG_IF(FATAL, im.empty()) << "read image " << impath << "failed!";
        double currtime = vImageList[imageIdx].timeStamp*e;

        const double T1 = cv::getTickCount();

        rovioNode.imgCallback0(im,currtime);

        while(vIMUList[imuIdx]._t < vImageList[imageIdx+1].timeStamp)//TODO:这里最后结束的时候逻辑有问题
        {
            double imutime = vIMUList[imuIdx]._t*e;
            rovioNode.imuCallback(vIMUList[imuIdx], imutime);
            imuIdx++;
        }

        const double T2 = cv::getTickCount();
        double T = (T2-T1)/cv::getTickFrequency()*1000;//ms

        vTimeCost.push_back(T);
        LOG(INFO)<< "Current frame track time cost: " << T << "ms";
    }

    cout<< "vtimecost: "<<vTimeCost.size()<<endl;
    sort(vTimeCost.begin(),vTimeCost.end());
    float totaltime = 0;
    for(uint ni=0; ni<vImageList.size(); ni++)
    {
        totaltime+=vTimeCost[ni];
    }
    LOG(INFO) << "-------" << endl << endl;
    LOG(INFO) << "median tracking time: " << vTimeCost[vImageList.size()/2] << "ms";
    LOG(INFO) << "mean tracking time: " << totaltime/vImageList.size() << "ms";
    LOG(INFO) << "longest tracking time: " << vTimeCost[vTimeCost.size()-1] << "ms";

    google::ShutdownGoogleLogging();
    return 0;
}