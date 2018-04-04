//
// Created by zhuzunjie on 18-4-4.
//

#include <iostream>
#include <iomanip>

#include "opencv2/core/core.hpp"
#include "glog/logging.h"

//#include "FileReader.hpp"
#include "IMU/imudata.h"
#include "IMU/configparam.h"
#include "System.h"

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
    if(argc != 6)
    {
        cerr << endl << "Usage: ./project "
                        "path_to_ORBVOC.TXT"
                        "path_to_euroc.yaml"
                        "path_to_imu/data.csv"
                        "path_to_cam0/data.csv"
                        "path_to_cam0/data" << endl;
        return -1;
    }
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "../logs";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;





    //read image and imu data
    vector<ICell> vImageList;
    loadImageList(argv[4], vImageList);
    vector<ORB_SLAM2::IMUData> vIMUList;
    loadIMUFile(argv[3],vIMUList);

    LOG(INFO)<<"pass";
    // 预处理数据,滤波器确实不需要初始同步，但这里的初始同步还是先放着。
    int imageIdx=0;
    int imuIdx=0;
    cout<<vImageList.size()<<endl;
    cout<<vIMUList.size()<<endl;
//    initData(vImageList,vIMUList,imageIdx,imuIdx);

    uint startImuIdx = 0;
    uint startImageIdx = 0;
    // 剔除初始的冗余IMU数据
    while(true)
    {
        if(vIMUList[startImuIdx]._t >= vImageList[0].timeStamp)
            break;

        startImuIdx++;
    }

    // 剔除初始的冗余Image数据
    while (true)
    {
        if (vIMUList[0]._t <= vImageList[startImageIdx].timeStamp)
            break;

        startImageIdx++;
    }
    // 将IMU和图片时间戳对齐
    while(true)
    {
        if(vIMUList[startImuIdx]._t >= vImageList[startImageIdx].timeStamp)
            break;

        startImuIdx++;
    }
    imageIdx = startImageIdx;
    imuIdx = startImuIdx;
    cout<<imageIdx<<endl;
    cout<<imuIdx<<endl;


    //rovio init
    string filter_config = "/home/zhuzunjie/Projects/RT-NOROS-VIORB/LearnVIORB_NOROS/config/rovio.info";
    std::shared_ptr<mtFilter> mpFilter(new mtFilter);
    mpFilter->readFromInfo(filter_config);
    std::string camera_config = "/home/zhuzunjie/Projects/RT-NOROS-VIORB/LearnVIORB_NOROS/config/euroc_cam0.yaml";
    mpFilter->cameraCalibrationFile_[0] = camera_config;
    mpFilter->refreshProperties();
    rovio::RovioNode<mtFilter> rovioNode(mpFilter);
    rovioNode.makeTest();

    //orbslam init
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::ConfigParam config(argv[2]);



    Eigen::Matrix3d rot; //groundtruth to camera
    rot << 0,0,-1,
            -1,0,0,
            0,1,0;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    // beijing 9.8012
    const double g3dm = 9.8012;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    string imdir = argv[5];
    vector<double> vTimeCost;///<记录每帧时间
    double e = pow(10.0, -9);

    for (int id = 0; id < imuIdx; ++id) {
        rovioNode.imuCallback(vIMUList[id],vIMUList[id]._t*e);
    }
    for (; imageIdx < vImageList.size()-10; ++imageIdx)
    {
        //load image
        LOG(INFO) << "<<<<<<<<<<<<<LOAD IMAGE " << imageIdx << " : " << vImageList[imageIdx].imgName;
        string impath = imdir+"/"+vImageList[imageIdx].imgName;
        cv::Mat im = cv::imread(impath,0);
        LOG_IF(FATAL, im.empty()) << "read image " << impath << "failed!";
        double currtime = vImageList[imageIdx].timeStamp*e;

        const double T1 = cv::getTickCount();

        rovioNode.imgCallback0(im,currtime);
//        Eigen::Vector3d pos = rovioNode.imuOutput_.WrWB();
//        SLAM.DrawRovioPos(Eigen::Vector3d(pos[1],-pos[2],-pos[0]));
        std::vector<ORB_SLAM2::IMUData> vimuData;
        while(vIMUList[imuIdx]._t < vImageList[imageIdx+1].timeStamp)//TODO:这里最后结束的时候逻辑有问题
        {
            double imutime = vIMUList[imuIdx]._t * e;
            rovioNode.imuCallback(vIMUList[imuIdx], imutime);

            Eigen::Vector3d pos = rovioNode.imuOutput_.WrWB();
            Eigen::Quaterniond qr(rovioNode.imuOutput_.qBW().x(),rovioNode.imuOutput_.qBW().y(),rovioNode.imuOutput_.qBW().z(),rovioNode.imuOutput_.qBW().w());
            // only need related transformation.

            if(bAccMultiply98)
            {
                vIMUList[imuIdx]._a(0) *= g3dm;
                vIMUList[imuIdx]._a(1) *= g3dm;
                vIMUList[imuIdx]._a(2) *= g3dm;
            }
            vIMUList[imuIdx]._t = imutime; //时间转为秒
            ORB_SLAM2::IMUData imudata(vIMUList[imuIdx]._g(0),vIMUList[imuIdx]._g(1),vIMUList[imuIdx]._g(2),
                                       vIMUList[imuIdx]._a(0),vIMUList[imuIdx]._a(1),vIMUList[imuIdx]._a(2),
                                       (double)vIMUList[imuIdx]._t);
            vimuData.push_back(imudata);
            imuIdx++;
        }
        vImageList[imageIdx].timeStamp = currtime;
        SLAM.TrackMonoVI(im, vimuData, currtime);

        const double T2 = cv::getTickCount();
        double T = (T2-T1)/cv::getTickFrequency()*1000;//ms

        vTimeCost.push_back(T);
        LOG(INFO)<< "Current frame track time cost: " << T << "ms";


        cv::waitKey(1);
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

    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");
    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

    google::ShutdownGoogleLogging();
    return 0;
}