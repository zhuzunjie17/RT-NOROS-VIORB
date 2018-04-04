/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../include/System.h"
#include "FileReader.h"
#include "../src/IMU/imudata.h"
#include "../src/IMU/configparam.h"

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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::ConfigParam config(argv[2]);
    string imdir = argv[5];

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    // beijing 9.8012
    const double g3dm = 9.8012;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    std::vector<ORB_SLAM2::IMUData> allimuData;
    std::vector<ICell> iListData;
    loadIMUFile(argv[3],allimuData);
    loadImageList(argv[4],iListData);

    // 数据初始同步
    uint imageIdx=0;
    uint imuIdx=0;

//    synInit(iListData,allimuData,imageIdx,imuIdx);

    uint startImuIdx = 0;
    uint startImageIdx = 0;
    // 剔除初始的冗余IMU数据
    while(true)
    {
        if(allimuData[startImuIdx]._t >= iListData[0].timeStamp)
            break;

        startImuIdx++;
    }

    // 剔除初始的冗余Image数据
    while (true)
    {
        if (allimuData[0]._t <= iListData[startImageIdx].timeStamp)
            break;

        startImageIdx++;
    }
    // 将IMU和图片时间戳对齐
    while(true)
    {
        if(allimuData[startImuIdx]._t >= iListData[startImageIdx].timeStamp)
            break;

        startImuIdx++;
    }
    imageIdx = startImageIdx;
    imuIdx = startImuIdx;
    double e = pow(10.0,-9);
	vector<double > vTimesTrack;

    for(;imageIdx<iListData.size();++imageIdx)
    {
        //捆绑两帧图像间的imu数据
        std::vector<ORB_SLAM2::IMUData> vimuData;
		while(allimuData[imuIdx]._t < iListData[imageIdx+1].timeStamp)
		{
			if(bAccMultiply98)
			{
				allimuData[imuIdx]._a(0) *= g3dm;
				allimuData[imuIdx]._a(1) *= g3dm;
				allimuData[imuIdx]._a(2) *= g3dm;
			}
			allimuData[imuIdx]._t = allimuData[imuIdx]._t*e; //时间转为秒
			ORB_SLAM2::IMUData imudata(allimuData[imuIdx]._g(0),allimuData[imuIdx]._g(1),allimuData[imuIdx]._g(2),
						                allimuData[imuIdx]._a(0),allimuData[imuIdx]._a(1),allimuData[imuIdx]._a(2),
                                        (double)allimuData[imuIdx]._t);
			vimuData.push_back(imudata);
			imuIdx++;
        }

        cout<<"<<<<<<<<<<<<<LOAD IMAGE " << imageIdx << " : " << iListData[imageIdx].imgName<<endl;
        string impath = imdir +"/"+iListData[imageIdx].imgName;
        cv::Mat im = cv::imread(impath,0);
        if(im.empty())
        {
            cerr << "load image failed! image path: " << impath <<endl;
            return -1;
        }
        iListData[imageIdx].timeStamp = iListData[imageIdx].timeStamp*e;

		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	    SLAM.TrackMonoVI(im, vimuData, iListData[imageIdx].timeStamp);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
		vTimesTrack.push_back(ttrack);
		cout<<"timecost: "<<ttrack<<endl;
		cv::waitKey(1);
     }
     // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(uint ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size() << endl;

    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath+"KeyFrameNavStateTrajectory.txt");
    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();
    return 0;
}