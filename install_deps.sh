#!/bin/sh
## Dependencies for Thunderbots@Home Software
## Used for updating and installing the distros. 
ROS_DISTRO="hydro"

sudo apt-get install openni-dev
sudo apt-get install ros-${ROS_DISTRO}-joystick-drivers
sudo apt-get install ros-${ROS_DISTRO}-joy
sudo apt-get install ros-${ROS_DISTRO}-pcl-conversions
sudo apt-get install ros-${ROS_DISTRO}-pcl-ros
sudo apt-get install ros-${ROS_DISTRO}-opencv2
sudo apt-get install ros-${ROS_DISTRO}-image-transport
sudo apt-get install ros-${ROS_DISTRO}-turtlebot-apps
sudo apt-get install cmake python-catkin_pkg python-empy python-nose python-setuptools libgtest-dev build-essential
sudo apt-get install ros-${ROS_DISTRO}-catkin
sudo apt-get install ros-${ROS_DISTRO}-turtlebot-follower
sudo apt-get install ros-${ROS_DISTRO}-turtlebot-msgs

## add sixad for using ps3 controller
sudo apt-add-repository ppa:falk-t-j/qtsixa
sudo apt-get update
sudo apt-get install sixad


