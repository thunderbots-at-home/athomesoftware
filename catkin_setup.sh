#!/bin/sh


## From wiki.ros.org/catkin_tutorials/create_a_workspace
source /opt/ros/hydro/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make

## ROS environment variables for workspace are available
## when opening a new terminal
echo "source ../devel/setup.sh" >> ~/.bashrc
