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
include CMakeFiles/runeth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/runeth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/runeth.dir/flags.make

CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o: CMakeFiles/runeth.dir/flags.make
CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o: ../Examples/MonoVIO/runeth.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o -c /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/Examples/MonoVIO/runeth.cc

CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/Examples/MonoVIO/runeth.cc > CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.i

CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/Examples/MonoVIO/runeth.cc -o CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.s

CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.requires:

.PHONY : CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.requires

CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.provides: CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.requires
	$(MAKE) -f CMakeFiles/runeth.dir/build.make CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.provides.build
.PHONY : CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.provides

CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.provides.build: CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o


# Object files for target runeth
runeth_OBJECTS = \
"CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o"

# External object files for target runeth
runeth_EXTERNAL_OBJECTS =

../Examples/MonoVIO/runeth: CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o
../Examples/MonoVIO/runeth: CMakeFiles/runeth.dir/build.make
../Examples/MonoVIO/runeth: ../lib/libORB_SLAM2.so
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_videostab.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_ts.a
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_superres.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_stitching.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_contrib.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_nonfree.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_ocl.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_gpu.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_photo.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_objdetect.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_legacy.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_video.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_ml.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_calib3d.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_features2d.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_highgui.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_imgproc.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_flann.so.2.4.13
../Examples/MonoVIO/runeth: /usr/local/lib/libopencv_core.so.2.4.13
../Examples/MonoVIO/runeth: /home/cristin/Documents/Thirdparty/Pangolin-master/build/src/libpangolin.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libpython2.7.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libavcodec.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libavformat.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libavutil.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libswscale.so
../Examples/MonoVIO/runeth: /usr/lib/libOpenNI.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libpng.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libz.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libjpeg.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libtiff.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libIlmImf.so
../Examples/MonoVIO/runeth: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/MonoVIO/runeth: ../Thirdparty/g2o/lib/libg2o.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libcholmod.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libamd.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libcolamd.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libcamd.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libccolamd.so
../Examples/MonoVIO/runeth: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../Examples/MonoVIO/runeth: CMakeFiles/runeth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/MonoVIO/runeth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/runeth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/runeth.dir/build: ../Examples/MonoVIO/runeth

.PHONY : CMakeFiles/runeth.dir/build

CMakeFiles/runeth.dir/requires: CMakeFiles/runeth.dir/Examples/MonoVIO/runeth.cc.o.requires

.PHONY : CMakeFiles/runeth.dir/requires

CMakeFiles/runeth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/runeth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/runeth.dir/clean

CMakeFiles/runeth.dir/depend:
	cd /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug /home/cristin/projects/RT_NOROS_VIORB/LearnVIORB_NOROS/cmake-build-debug/CMakeFiles/runeth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/runeth.dir/depend

