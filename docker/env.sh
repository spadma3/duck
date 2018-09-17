#!/bin/bash

if [[ $(ping -c 1 $ROS_MASTER) -eq 0 ]]; then
    export ROS_MASTER_URI="http://$ROS_MASTER:11311/"
else
    IP=$(ifconfig wlan0 | grep "inet " | awk -F'[: ]+' '{ print $4 }')
    export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://$IP:11311/"}
    export ROS_IP=${IP}
fi

source /home/software/environment.sh
export DUCKIEFLEET_ROOT=/data/config
export VEHICLE_NAME=$HOSTNAME
cat misc/duckie.art

/home/software/docker/init_config_defaults.sh
