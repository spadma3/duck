#!/bin/bash
echo "Inside entrypoint.sh"
echo "USER $USER"
echo "UID $UID"
export HOME=/home
echo "HOME $HOME"
export COLUMNS=160
export NODE_PATH=/project/node_modules
echo "Activating ROS"
source /opt/ros/kinetic/setup.sh
echo "Activating virtual env"
source /project/deploy/bin/activate
echo "Now executing command $@"
exec "$@"
