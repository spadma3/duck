#!/bin/bash

if [ -z "$ROS_MASTER" ]; then
    IP=$(hostname -I | cut -d " " -f 1)
    echo "No \$ROS_MASTER was passed, defaulting to localhost/$(hostname)/$IP"
    export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://$IP:11311/"}
    echo "Setting ROS_MASTER_URI to $ROS_MASTER_URI"
    export ROS_IP=${IP}
elif [[ $(ping -c 1 $ROS_MASTER) -eq 0 ]]; then
    export ROS_MASTER_URI="http://$ROS_MASTER:11311/"
elif [[ $(ping -c 1 "$ROS_MASTER.local") -eq 0 ]]; then
    export ROS_MASTER_URI="http://$ROS_MASTER.local:11311/"
else
    echo -e "Neither $ROS_MASTER nor $ROS_MASTER.local are reachable, aborting."
    exit 1
fi

source /home/software/environment.sh
export DUCKIEFLEET_ROOT=/data/config
export VEHICLE_NAME=$HOSTNAME
cat misc/duckie.art

/home/software/docker/init_config_defaults.sh
