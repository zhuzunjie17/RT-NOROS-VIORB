CMAKE_MINIMUM_REQUIRED(VERSION 3.4)
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

FIND_PACKAGE(kindr REQUIRED)
FIND_PACKAGE(LWF REQUIRED)
FIND_PACKAGE(Glog REQUIRED)
FIND_PACKAGE(PkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)

INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src

        ${Pangolin_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}

        ${kindr_INCLUDE_DIRS}
        ${LWF_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${YamlCpp_INCLUDE_DIRS}
)

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

FILE(GLOB rovio_headers ${PROJECT_SOURCE_DIR}/include/rovio/*.hpp ${PROJECT_SOURCE_DIR}/include/rovio/CoordinateTransform/*.hpp)
FILE(GLOB orbslam_headers ${PROJECT_SOURCE_DIR}/include/*.h )
FILE(GLOB orbslam_imu_headers ${PROJECT_SOURCE_DIR}/src/IMU/*.h)
FILE(GLOB orbslam_sources ${PROJECT_SOURCE_DIR}/src/*.cc)
FILE(GLOB orbslam_imu_sources ${PROJECT_SOURCE_DIR}/src/IMU/*.cpp)

SET(ROVIO_HEADERS
        ${rovio_headers}
        )
SET(ROVIO_SOURCES
        src/rovio/Camera.cpp
        src/rovio/FeatureCoordinates.cpp
        src/rovio/FeatureDistance.cpp
        )
SET(ORBSLAM_HEADERS
        ${orbslam_headers}
        )
SET(ORBSLAM_SOURCES
        ${orbslam_sources}
        )
SET(ORBSLAM_IMU_HEADERS
        ${orbslam_imu_headers}
        )
SET(ORBSLAM_IMU_SOURCES
        ${orbslam_imu_sources}
        )


SOURCE_GROUP(Header\ Files\\Rovio FILES ${ROVIO_HEADERS})
SOURCE_GROUP(Header\ Files\\ORBSLAM FILES ${ORBSLAM_HEADERS})
SOURCE_GROUP(Header\ Files\\IMU FILES ${ORBSLAM_IMU_HEADERS})
SOURCE_GROUP(Source\ Files\\Rovio FILES ${ROVIO_SOURCES})
SOURCE_GROUP(Source\ Files\\ORBSLAM FILES ${ORBSLAM_SOURCES})
SOURCE_GROUP(Source\ Files\\IMU FILES ${ORBSLAM_IMU_SOURCES})


ADD_LIBRARY(${PROJECT_NAME} SHARED
        ${ROVIO_HEADERS}
        ${ORBSLAM_HEADERS}
        ${ORBSLAM_IMU_HEADERS}
        ${ROVIO_SOURCES}
        ${ORBSLAM_SOURCES}
        ${ORBSLAM_IMU_SOURCES}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so

        ${YamlCpp_LIBRARIES}
        ${GLOG_LIBRARIES}
)

ADD_EXECUTABLE(vio
Examples/MonoVIO/vio.cc)
TARGET_LINK_LIBRARIES(vio ${PROJECT_NAME})

ADD_EXECUTABLE(rovio
        Examples/MonoVIO/rovio_noros.cpp)
TARGET_LINK_LIBRARIES(rovio ${PROJECT_NAME})

ADD_EXECUTABLE(orb_with_rovio
        Examples/MonoVIO/orb_with_rovio.cpp)
TARGET_LINK_LIBRARIES(orb_with_rovio ${PROJECT_NAME})

ADD_EXECUTABLE(ZR300
        Examples/MonoVIO/zr300_rovio_orb.cpp)
TARGET_LINK_LIBRARIES(ZR300 ${PROJECT_NAME})