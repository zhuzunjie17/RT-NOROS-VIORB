# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/cristin/clion-2017.2.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/cristin/clion-2017.2.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/rosvio.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rosvio.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rosvio.dir/flags.make

CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o: CMakeFiles/rosvio.dir/flags.make
CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o: ../Examples/ROS/ORB_VIO/src/ros_vio.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o -c /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/Examples/ROS/ORB_VIO/src/ros_vio.cc

CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/Examples/ROS/ORB_VIO/src/ros_vio.cc > CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.i

CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/Examples/ROS/ORB_VIO/src/ros_vio.cc -o CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.s

CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.requires:

.PHONY : CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.requires

CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.provides: CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.requires
	$(MAKE) -f CMakeFiles/rosvio.dir/build.make CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.provides.build
.PHONY : CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.provides

CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.provides.build: CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o


# Object files for target rosvio
rosvio_OBJECTS = \
"CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o"

# External object files for target rosvio
rosvio_EXTERNAL_OBJECTS =

../Examples/ROS/ORB_VIO/src/rosvio: CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o
../Examples/ROS/ORB_VIO/src/rosvio: CMakeFiles/rosvio.dir/build.make
../Examples/ROS/ORB_VIO/src/rosvio: ../lib/libORB_SLAM2.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_videostab.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_ts.a
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_superres.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_stitching.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_contrib.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_nonfree.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_ocl.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_gpu.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_photo.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_objdetect.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_legacy.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_video.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_ml.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_calib3d.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_features2d.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_highgui.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_imgproc.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_flann.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /usr/local/lib/libopencv_core.so.2.4.13
../Examples/ROS/ORB_VIO/src/rosvio: /home/cristin/Documents/Thirdparty/Pangolin-master/build/src/libpangolin.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libavcodec.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libavformat.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libavutil.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libswscale.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/libOpenNI.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/ROS/ORB_VIO/src/rosvio: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/ROS/ORB_VIO/src/rosvio: ../Thirdparty/g2o/lib/libg2o.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libcholmod.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libamd.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libcolamd.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libcamd.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libccolamd.so
../Examples/ROS/ORB_VIO/src/rosvio: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../Examples/ROS/ORB_VIO/src/rosvio: CMakeFiles/rosvio.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/ROS/ORB_VIO/src/rosvio"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rosvio.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rosvio.dir/build: ../Examples/ROS/ORB_VIO/src/rosvio

.PHONY : CMakeFiles/rosvio.dir/build

CMakeFiles/rosvio.dir/requires: CMakeFiles/rosvio.dir/Examples/ROS/ORB_VIO/src/ros_vio.cc.o.requires

.PHONY : CMakeFiles/rosvio.dir/requires

CMakeFiles/rosvio.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rosvio.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rosvio.dir/clean

CMakeFiles/rosvio.dir/depend:
	cd /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug/CMakeFiles/rosvio.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rosvio.dir/depend

