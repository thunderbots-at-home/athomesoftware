# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/marco/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marco/catkin_ws/src

# Include any dependencies generated for this target.
include vision/CMakeFiles/object_classification.dir/depend.make

# Include the progress variables for this target.
include vision/CMakeFiles/object_classification.dir/progress.make

# Include the compile flags for this target's objects.
include vision/CMakeFiles/object_classification.dir/flags.make

vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o: vision/CMakeFiles/object_classification.dir/flags.make
vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o: vision/src/ObjectClassification.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marco/catkin_ws/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o"
	cd /home/marco/catkin_ws/src/vision && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o -c /home/marco/catkin_ws/src/vision/src/ObjectClassification.cpp

vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.i"
	cd /home/marco/catkin_ws/src/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marco/catkin_ws/src/vision/src/ObjectClassification.cpp > CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.i

vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.s"
	cd /home/marco/catkin_ws/src/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marco/catkin_ws/src/vision/src/ObjectClassification.cpp -o CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.s

vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.requires:
.PHONY : vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.requires

vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.provides: vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.requires
	$(MAKE) -f vision/CMakeFiles/object_classification.dir/build.make vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.provides.build
.PHONY : vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.provides

vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.provides.build: vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o

# Object files for target object_classification
object_classification_OBJECTS = \
"CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o"

# External object files for target object_classification
object_classification_EXTERNAL_OBJECTS =

/home/marco/catkin_ws/devel/lib/vision/object_classification: vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libroscpp.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/libboost_signals-mt.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/libboost_filesystem-mt.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/libboost_system-mt.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libcpp_common.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/librostime.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/libboost_date_time-mt.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/libboost_thread-mt.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/librosconsole.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/libboost_regex-mt.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /usr/lib/liblog4cxx.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libcv_bridge.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_calib3d.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_contrib.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_core.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_features2d.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_flann.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_gpu.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_highgui.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_imgproc.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_legacy.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_ml.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_nonfree.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_objdetect.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_photo.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_stitching.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_superres.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_ts.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_video.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_videostab.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_calib3d.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_contrib.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_core.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_features2d.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_flann.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_gpu.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_highgui.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_imgproc.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_legacy.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_ml.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_nonfree.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_objdetect.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_photo.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_stitching.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_superres.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_video.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_videostab.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: /opt/ros/hydro/lib/libopencv_ts.so
/home/marco/catkin_ws/devel/lib/vision/object_classification: vision/CMakeFiles/object_classification.dir/build.make
/home/marco/catkin_ws/devel/lib/vision/object_classification: vision/CMakeFiles/object_classification.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/marco/catkin_ws/devel/lib/vision/object_classification"
	cd /home/marco/catkin_ws/src/vision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/object_classification.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision/CMakeFiles/object_classification.dir/build: /home/marco/catkin_ws/devel/lib/vision/object_classification
.PHONY : vision/CMakeFiles/object_classification.dir/build

vision/CMakeFiles/object_classification.dir/requires: vision/CMakeFiles/object_classification.dir/src/ObjectClassification.cpp.o.requires
.PHONY : vision/CMakeFiles/object_classification.dir/requires

vision/CMakeFiles/object_classification.dir/clean:
	cd /home/marco/catkin_ws/src/vision && $(CMAKE_COMMAND) -P CMakeFiles/object_classification.dir/cmake_clean.cmake
.PHONY : vision/CMakeFiles/object_classification.dir/clean

vision/CMakeFiles/object_classification.dir/depend:
	cd /home/marco/catkin_ws/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marco/catkin_ws/src /home/marco/catkin_ws/src/vision /home/marco/catkin_ws/src /home/marco/catkin_ws/src/vision /home/marco/catkin_ws/src/vision/CMakeFiles/object_classification.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision/CMakeFiles/object_classification.dir/depend

