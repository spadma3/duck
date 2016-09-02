#!/usr/bin/env bash


# To be called by joystick-daemon.py whenever the start button is pressed on a joystick
# To be run as user=ubuntu

source /home/ubuntu/duckietown/environment.sh; 
source /home/ubuntu/duckietown/set_ros_master.sh; 
roslaunch duckietown joystick.launch veh:=cepillo;

exit 0 
