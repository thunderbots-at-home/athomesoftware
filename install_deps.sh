#!/bin/sh
## Dependencies for Thunderbots@Home Software
## Used for updating and installing the distros. 
ROS_DISTRO="hydro"
INSTALLATION_DIR="~/catkin_ws/src/athomesoftware/"
mkdir ${INSTALLATION_DIR}/dependencies
DEPENDENCIES_DIR="${INSTALLATION_DIR}/dependencies/"

## Kinect related dependencies
## sudo apt-get install openni-dev DO NOT UNCOMMENT THIS IT IS EVIL
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install freeglut3-dev
sudo apt-get install doxygen
sudo apt-get install graphviz
sudo apt-get install libudev-dev

## ROS Packages
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
sudo apt-get install ros-${ROS_DISTRO}-urdfdom
sudo apt-get install ros-${ROS_DISTRO}-hokuyo-node
sudo apt-get install ros-${ROS_DISTRO}-hector-slam

#Installing Gazebo Simulator (This installs the most recent version of gazebo)
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu precise main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo-current

## SIXAD for ps3 controller
sudo apt-add-repository ppa:falk-t-j/qtsixa
sudo apt-get update
sudo apt-get install sixad

# Installing pylearn2 dependencies
sudo apt-get install python3
sudo apt-get install python-scipy
sudo apt-get install python-numpy
sudo apt-get install python-pip
pip install git+http://github.com/Theano/Theano.git
svn checkout http://svn.pyyaml.org/pyyaml/trunk ${DEPENDENCIES_DIR}/pyyaml-trunk






