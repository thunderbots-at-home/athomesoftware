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
CMAKE_SOURCE_DIR = /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning

# Include any dependencies generated for this target.
include CMakeFiles/confusionmatrix.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/confusionmatrix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/confusionmatrix.dir/flags.make

CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o: CMakeFiles/confusionmatrix.dir/flags.make
CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o: src/ConfusionMatrix.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o -c /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning/src/ConfusionMatrix.cpp

CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning/src/ConfusionMatrix.cpp > CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.i

CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning/src/ConfusionMatrix.cpp -o CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.s

CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.requires:
.PHONY : CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.requires

CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.provides: CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.requires
	$(MAKE) -f CMakeFiles/confusionmatrix.dir/build.make CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.provides.build
.PHONY : CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.provides

CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.provides.build: CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o

# Object files for target confusionmatrix
confusionmatrix_OBJECTS = \
"CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o"

# External object files for target confusionmatrix
confusionmatrix_EXTERNAL_OBJECTS =

devel/lib/libconfusionmatrix.so: CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o
devel/lib/libconfusionmatrix.so: CMakeFiles/confusionmatrix.dir/build.make
devel/lib/libconfusionmatrix.so: CMakeFiles/confusionmatrix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library devel/lib/libconfusionmatrix.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/confusionmatrix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/confusionmatrix.dir/build: devel/lib/libconfusionmatrix.so
.PHONY : CMakeFiles/confusionmatrix.dir/build

CMakeFiles/confusionmatrix.dir/requires: CMakeFiles/confusionmatrix.dir/src/ConfusionMatrix.cpp.o.requires
.PHONY : CMakeFiles/confusionmatrix.dir/requires

CMakeFiles/confusionmatrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/confusionmatrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/confusionmatrix.dir/clean

CMakeFiles/confusionmatrix.dir/depend:
	cd /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning /home/marco/catkin_ws/src/athomesoftware/talos_machinelearning/CMakeFiles/confusionmatrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/confusionmatrix.dir/depend

