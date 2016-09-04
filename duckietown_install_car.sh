#!/usr/bin/env bash
set -e
set -x

# Install some packages that were missed in v1.1. Not necessary anymore in v1.2
sudo apt-get install ros-indigo-{tf-conversions,cv-bridge,image-transport,camera-info-manager,theora-image-transport,joy,image-proc} -y
sudo apt-get install ros-indigo-compressed-image-transport -y
sudo apt-get install libyaml-cpp-dev -y

# # packages for the IMU
sudo apt-get install ros-indigo-phidgets-drivers
sudo apt-get install ros-indigo-imu-complementary-filter ros-indigo-imu-filter-madgwick

# Install the joystick daemon for executing bash from joystick button presses
sudo $HOME/duckietown/catkin_ws/src/f11-no_laptop/install.sh

# # scipy for lane-filter
# sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran
# sudo pip install scipy --upgrade
