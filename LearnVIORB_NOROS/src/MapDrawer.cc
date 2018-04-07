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

#include "MapDrawer.h"
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap),mbInitTime(true) {


    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    mbGroundTruth = fSettings["Viewer.groundtruth"];
    fSettings["Viewer.GTfile"] >> mGTfile;
    mCount = 0;
    mInit = 0;
    mbInitTime = false;
    mRovioInitPos.setZero(3);
    Eigen::Matrix3d R_BS;
    Eigen::Vector3d t_BS;
    cout << mGTfile << endl;
    if (mbGroundTruth) {
        mbInitTime = true;
        loadGTFile(mGTfile.c_str(), mGTData);

        cv::FileNode Tbs_ = fSettings["T.BS"];
        R_BS << Tbs_[0], Tbs_[1], Tbs_[2],
                Tbs_[4], Tbs_[5], Tbs_[6],
                Tbs_[8], Tbs_[9], Tbs_[10];

        t_BS << Tbs_[3], Tbs_[7], Tbs_[11];

        for(int i=0; i<mGTData.size(); i++) {
            mGTData[i].position = R_BS * mGTData[i].position + t_BS; //因为是相对pose，所以位移差应该没用
            Eigen::Vector3d p = R_BS * mGTData[i].position;
            mGTData[i].position[0] = p[1];
            mGTData[i].position[1] = -p[2];
            mGTData[i].position[2] = -p[0];

        }
    }
}

void MapDrawer::GetInitTime(double t)
{
	if(mbInitTime)
	{
	    //将groundtruth坐标对齐到相机初始化成功坐标位置
		unique_lock<mutex> lock(mMutexInit);
		while(mGTData[mInit+1].timeStamp <= t)
			mInit++;
		mbInitTime = false;
		Eigen::Vector3d pos = mGTData[mInit].position;//将GT轨迹与当前初始化的轨迹同步（初始化位置，和外参）
        for(int i=0; i<mGTData.size(); i++)
            mGTData[i].position -= pos;

        //将rovio轨迹坐标对齐到相机初始化成功坐标位置
        unique_lock<mutex> lock1(mMutexRovio);
        mRovioInitPos = mRovioPos[mRovioPos.size()-1];
        for(int i = 0; i<mRovioPos.size(); ++i)
            mRovioPos[i] -= mRovioInitPos;
    }
}

void MapDrawer::TimeSet(double t)
{
	unique_lock<mutex> lock(mMutexTime);
	mTime = t;
}

void MapDrawer::DrawGroundTruth()
{
//	if(mpMap->KeyFramesInMap())
	    while(mGTData[mCount].timeStamp < mTime)
		    mCount++;
	glPushMatrix();
	glLineWidth(1);
	glColor3f(1.0f,0.0f,0.0f);
	glBegin(GL_LINES);
	Eigen::Vector3d pos(mGTData[0].position);
	for(int i=0; i<mCount; i++)
	{
		glVertex3d(pos[0],pos[1],pos[2]);
		pos = mGTData[i].position;
		glVertex3d(pos[0],pos[1],pos[2]);
	}
	glEnd();
	glPopMatrix();
}

void MapDrawer::DrawRovio()
{
    //不知道这里要不要加锁
    if(!mRovioPos.empty()) {
        glPushMatrix();
        glLineWidth(1);
        glColor3f(1.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        Eigen::Vector3d pos(mRovioPos[0]);
        for (int i = 0; i < mRovioPos.size(); i++) {
            glVertex3d(pos[0], pos[1], pos[2]);
            pos = mRovioPos[i];
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
        glPopMatrix();
    }
}

void MapDrawer::UpdateRovio(const Eigen::Vector3d& pos) {
    unique_lock<mutex> lock(mMutexRovio);
    mRovioPos.push_back(pos - mRovioInitPos);
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    //glColor3f(0.0f,1.0f,0.0f);
    glColor3f(1.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
