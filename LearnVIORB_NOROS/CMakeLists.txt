CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release )
ENDIF()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -march=native -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(OpenCV 2.4 REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Pangolin REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)

INCLUDE_DIRECTORIES(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR} /usr/local/include/eigen3
${Pangolin_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/FileReader.cc

include/FileReader.h
include/Frame.h
include/KeyFrame.h
include/Tracking.h
include/LocalMapping.h

src/IMU/configparam.h
src/IMU/configparam.cpp

src/IMU/imudata.h
src/IMU/imudata.cpp
src/IMU/IMUPreintegrator.h
src/IMU/IMUPreintegrator.cpp
src/IMU/so3.cpp
src/IMU/so3.h
src/IMU/NavState.h
src/IMU/NavState.cpp

src/IMU/g2otypes.h
src/IMU/g2otypes.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
cholmod
${CHOLMOD_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
)

## Build examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/ROS/ORB_VIO/src)
# 
# add_executable(rosvio
# Examples/ROS/ORB_VIO/src/ros_vio.cc)
# target_link_libraries(rosvio ${PROJECT_NAME})
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/MonoVIO)

add_executable(vio
Examples/MonoVIO/vio.cc)
target_link_libraries(vio ${PROJECT_NAME})

# add_executable(runeth
# Examples/MonoVIO/runeth.cc)
# target_link_libraries(runeth ${PROJECT_NAME})
# 
#add_executable(deblur
#Examples/DeblurVIO/Deblur.cpp)
#target_link_libraries(deblur ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc
# Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})

## Build tools
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary
#tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME})
