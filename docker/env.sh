#!/bin/bash

MY_IP=$(hostname -I | cut -d " " -f 1)
export ROS_IP=${MY_IP}
echo "Setting ROS_IP to host IP, which is $ROS_IP"

if [ -z "$ROS_MASTER" ]; then
    ROS_MASTER_IP=${MY_IP}
    echo "No \$ROS_MASTER was passed, defaulting to localhost/$(hostname)/$IP"
    export ROS_MASTER_URI=${ROS_MASTER_URI:-"http://$IP:11311/"}
elif ping -c 1 $ROS_MASTER; then
    ROS_MASTER_IP=$(ping -c1 $ROS_MASTER | grep -oP 'PING.*?\(\K[^)]+')
    export ROS_MASTER_URI="http://$ROS_MASTER:11311/"
elif ping -c 1 "$ROS_MASTER.local"; then
    ROS_MASTER_IP=$(ping -c1 $ROS_MASTER.local | grep -oP 'PING.*?\(\K[^)]+')
    export ROS_MASTER_URI="http://$ROS_MASTER.local:11311/"
else
    echo -e "Neither $ROS_MASTER nor $ROS_MASTER.local are reachable, aborting."
    exit 1
fi

echo "Setting ROS_MASTER_URI to $ROS_MASTER_URI, ROS_MASTER_IP is $ROS_MASTER_IP"

echo "$ROS_MASTER_IP $ROS_MASTER $ROS_MASTER.local" >> /etc/hosts

source /home/software/environment.sh
export DUCKIEFLEET_ROOT=/data/config
export VEHICLE_NAME=$HOSTNAME
cat misc/duckie.art

/home/software/docker/init_config_defaults.sh
