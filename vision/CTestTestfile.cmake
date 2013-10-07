# CMake generated Testfile for 
# Source directory: /home/marco/catkin_ws/src/vision
# Build directory: /home/marco/catkin_ws/src/vision
# 
# This file includes the relevent testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(_ctest_vision_gtest_vision-test "/home/marco/catkin_ws/src/vision/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/hydro/share/catkin/cmake/test/run_tests.py" "/home/marco/catkin_ws/src/vision/test_results/vision/gtest-vision-test.xml" "--return-code" "/home/marco/catkin_ws/src/vision/devel/lib/vision/vision-test --gtest_output=xml:/home/marco/catkin_ws/src/vision/test_results/vision/gtest-vision-test.xml")
SUBDIRS(gtest)
