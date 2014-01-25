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



