#!/usr/bin/env bash
set -e
set -x

# Install some packages that were missed in v1.1. Not necessary anymore in v1.2
sudo apt-get install ros-$ROS_DISTRO-{tf-conversions,cv-bridge,image-transport,camera-info-manager,theora-image-transport,joy,image-proc} -y
sudo apt-get install ros-$ROS_DISTRO-compressed-image-transport -y
sudo apt-get install libyaml-cpp-dev -y

# # packages for the IMU
sudo apt-get install ros-$ROS_DISTRO-phidgets-drivers
sudo apt-get install ros-$ROS_DISTRO-imu-complementary-filter ros-$ROS_DISTRO-imu-filter-madgwick

# # scipy for lane-filter
# sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran
# sudo pip install scipy --upgrade
