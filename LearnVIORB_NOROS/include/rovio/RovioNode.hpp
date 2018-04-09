/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <memory>
#include <mutex>
#include <queue>

#include <opencv2/core/core.hpp>
#include <glog/logging.h>
#include "IMU/imudata.h"
#include "rovio/RovioFilter.hpp"
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"

namespace rovio {

/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template<typename FILTER>
class RovioNode{
 public:
  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0,typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  typedef typename std::tuple_element<1,typename mtFilter::mtUpdates>::type mtPoseUpdate;
  mtPoseUpdate* mpPoseUpdate_;

  struct FilterInitializationState {
    FilterInitializationState()
        : WrWM_(V3D::Zero()),
          state_(State::WaitForInitUsingAccel) {}

    enum class State {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const {
      return isInitialized();
    }

    bool isInitialized() const {
      return (state_ == State::Initialized);
    }
  };
  FilterInitializationState init_state_;

  std::mutex m_filter_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput imuOutput_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;

  /** \brief Constructor
   */
  RovioNode(std::shared_ptr<mtFilter> mpFilter)
      :  mpFilter_(mpFilter), transformFeatureOutputCT_(&mpFilter->multiCamera_), landmarkOutputImuCT_(&mpFilter->multiCamera_) {
    mpImgUpdate_ = &std::get<0>(mpFilter_->mUpdates_);
    mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
  }

  /** \brief Destructor
   */
  virtual ~RovioNode(){}

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest(){
    mtFilterState* mpTestFilterState = new mtFilterState();
    *mpTestFilterState = mpFilter_->init_;
    mpTestFilterState->setCamera(&mpFilter_->multiCamera_);
    mtState& testState = mpTestFilterState->state_;
    unsigned int s = 2;
    testState.setRandom(s);
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    LWF::NormalVectorElement tempNor;
    for(int i=0;i<mtState::nMax_;i++){
      testState.CfP(i).camID_ = 0;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.CfP(i).set_nor(tempNor);
      testState.CfP(i).trackWarping_ = false;
      tempNor.setRandom(s);
      if(tempNor.getVec()(2) < 0){
        tempNor.boxPlus(Eigen::Vector2d(3.14,0),tempNor);
      }
      testState.aux().feaCoorMeas_[i].set_nor(tempNor,true);
      testState.aux().feaCoorMeas_[i].mpCamera_ = &mpFilter_->multiCamera_.cameras_[0];
      testState.aux().feaCoorMeas_[i].camID_ = 0;
    }
    testState.CfP(0).camID_ = mtState::nCam_-1;
    mpTestFilterState->fsm_.setAllCameraPointers();

    // Prediction
    std::cout << "Testing Prediction" << std::endl;
    mpFilter_->mPrediction_.testPredictionJacs(testState,predictionMeas_,1e-8,1e-6,0.1);

    // Update
    if(!mpImgUpdate_->useDirectMethod_){
      std::cout << "Testing Update (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
      for(int i=0;i<(std::min((int)mtState::nMax_,2));i++){
        testState.aux().activeFeature_ = i;
        testState.aux().activeCameraCounter_ = 0;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
        testState.aux().activeCameraCounter_ = mtState::nCam_-1;
        mpImgUpdate_->testUpdateJacs(testState,imgUpdateMeas_,1e-4,1e-5);
      }
    }

    // Testing CameraOutputCF and CameraOutputCF
    std::cout << "Testing cameraOutputCF" << std::endl;
    cameraOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing imuOutputCF" << std::endl;
    imuOutputCT_.testTransformJac(testState,1e-8,1e-6);
    std::cout << "Testing attitudeToYprCF" << std::endl;
    rovio::AttitudeToYprCT attitudeToYprCF;
    attitudeToYprCF.testTransformJac(1e-8,1e-6);

    // Testing TransformFeatureOutputCT
    std::cout << "Testing transformFeatureOutputCT" << std::endl;
    transformFeatureOutputCT_.setFeatureID(0);
    if(mtState::nCam_>1){
      transformFeatureOutputCT_.setOutputCameraID(1);
      transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);
    }
    transformFeatureOutputCT_.setOutputCameraID(0);
    transformFeatureOutputCT_.testTransformJac(testState,1e-8,1e-5);

    // Testing LandmarkOutputImuCT
    std::cout << "Testing LandmarkOutputImuCT" << std::endl;
    landmarkOutputImuCT_.setFeatureID(0);
    landmarkOutputImuCT_.testTransformJac(testState,1e-8,1e-5);

    // Getting featureOutput for next tests
    transformFeatureOutputCT_.transformState(testState,featureOutput_);
    if(!featureOutput_.c().isInFront()){
      featureOutput_.c().set_nor(featureOutput_.c().get_nor().rotated(QPD(0.0,1.0,0.0,0.0)),false);
    }

    // Testing FeatureOutputReadableCT
    std::cout << "Testing FeatureOutputReadableCT" << std::endl;
    featureOutputReadableCT_.testTransformJac(featureOutput_,1e-8,1e-5);

    // Testing pixelOutputCT
    rovio::PixelOutputCT pixelOutputCT;
    std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut errors due to the float precision)" << std::endl;
    pixelOutputCT.testTransformJac(featureOutput_,1e-4,1.0); // Reduces accuracy due to float and strong camera distortion

    // Testing ZeroVelocityUpdate_
    std::cout << "Testing zero velocity update" << std::endl;
    mpImgUpdate_->zeroVelocityUpdate_.testJacs();

    // Testing PoseUpdate
    if(!mpPoseUpdate_->noFeedbackToRovio_){
      std::cout << "Testing pose update" << std::endl;
      mpPoseUpdate_->testUpdateJacs(1e-8,1e-5);
    }

    delete mpTestFilterState;
  }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback(const ORB_SLAM2::IMUData& imu_msg, double time){
    std::lock_guard<std::mutex> lock(m_filter_);
    predictionMeas_.template get<mtPredictionMeas::_acc>() = imu_msg._a;
    predictionMeas_.template get<mtPredictionMeas::_gyr>() = imu_msg._g;
    if(init_state_.isInitialized()){
      mpFilter_->addPredictionMeas(predictionMeas_,time);
      updateAndPublish();
    } else {
      switch(init_state_.state_) {
        case FilterInitializationState::State::WaitForInitExternalPose: {
          std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
          mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, time);
          break;
        }
        case FilterInitializationState::State::WaitForInitUsingAccel: {
          std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
          mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),time);
          break;
        }
        default: {
          std::cout << "Unhandeld initialization type." << std::endl;
          abort();
          break;
        }
      }

      std::cout << std::setprecision(12);
      std::cout << "-- Filter: Initialized at t = " << time << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
  }

  /** \brief Image callback for the camera with ID 0
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback0(const cv::Mat& img, const double& currtime){
    std::lock_guard<std::mutex> lock(m_filter_);
    imgCallback(img,currtime,0);
  }

  /** \brief Image callback for the camera with ID 1
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback1(const cv::Mat& img, const double& currtime) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if(mtState::nCam_ > 1) imgCallback(img,currtime,0);
  }

  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(const cv::Mat& cv_img, const double& currtime, const int camID = 0){
    if(init_state_.isInitialized() && !cv_img.empty()){
      double msgTime = currtime;
      if(msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_){
        for(int i=0;i<mtState::nCam_;i++){
          if(imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i]){
            std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m" << std::endl;
          }
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img,true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if(imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid()){
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_,msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
        updateAndPublish();
      }
    }
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset() {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D& WrWM, const QPD& qMW) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!init_state_.isInitialized()) {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.WrWM_ = WrWM;
    init_state_.qMW_ = qMW;
    init_state_.state_ = FilterInitializationState::State::WaitForInitExternalPose;
  }

  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish(){
    if(init_state_.isInitialized()){
      // Execute the filter update.
      const double t1 = (double) cv::getTickCount();
      static double timing_T = 0;
      static int timing_C = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      double lastImageTime;
      if(std::get<0>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime)){
        mpFilter_->updateSafe(&lastImageTime);
      }
      const double t2 = (double) cv::getTickCount();
      int c2 = std::get<0>(mpFilter_->updateTimelineTuple_).measMap_.size();
      timing_T += (t2-t1)/cv::getTickFrequency()*1000;
      timing_C += c1-c2;
      bool plotTiming = false;
      if(plotTiming){
        LOG(INFO) << " == Filter Update: " << (t2-t1)/cv::getTickFrequency()*1000 << " ms for processing " << c1-c2 << " images, average: " << timing_T/timing_C;
      }
      if(mpFilter_->safe_.t_ > oldSafeTime){ // Publish only if something changed
        for(int i=0;i<mtState::nCam_;i++){
          if(!mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_){
            cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
            cv::waitKey(3);
          }
        }
        if(!mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_){
          cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
          cv::waitKey(3);
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
	    mtState& state = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        imuOutputCT_.transformState(state,imuOutput_);
        V3D POS = imuOutput_.WrWB();

        // Cout verbose for pose measurements
        if(mpImgUpdate_->verbose_){
          if(mpPoseUpdate_->inertialPoseIndex_ >=0){
            std::cout << "Transformation between inertial frames, IrIW, qWI: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_) << std::endl;
          }
          if(mpPoseUpdate_->bodyPoseIndex_ >=0){
            std::cout << "Transformation between body frames, MrMV, qVM: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_) << std::endl;
          }
        }

      }
    }
  }
};

}


#endif /* ROVIO_ROVIONODE_HPP_ */
