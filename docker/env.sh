#!/bin/bash

IP=$(hostname -I | cut -d " " -f 1)
export ROS_IP=${IP}


if [ -z "$ROS_MASTER" ]; then
    echo "No \$ROS_MASTER was passed, defaulting to localhost/$(hostname)/$IP"
    export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://$IP:11311/"}
elif ping -c 1 $ROS_MASTER; then
    export ROS_MASTER_URI="http://$ROS_MASTER:11311/"
elif ping -c 1 "$ROS_MASTER.local"; then
    export ROS_MASTER_URI="http://$ROS_MASTER.local:11311/"
else
    echo -e "Neither $ROS_MASTER nor $ROS_MASTER.local are reachable, aborting."
    exit 1
fi

echo "Setting ROS_MASTER_URI to $ROS_MASTER_URI"

echo "$IP $ROS_MASTER.local" >> /etc/hosts

source /home/software/environment.sh
export DUCKIEFLEET_ROOT=/data/config
export VEHICLE_NAME=$HOSTNAME
cat misc/duckie.art

/home/software/docker/init_config_defaults.sh
