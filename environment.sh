#!/bin/bash

[ -z "$DUCKIETOWN_ROOT" ] && { echo "Need to set DUCKIETOWN_ROOT"; exit 2; } 
[ -z "$HOSTNAME" ] && { echo "Need to set HOSTNAME."; exit 3; }

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

echo "Activating ROS..."
source /opt/ros/kinetic/setup.bash

echo "Setup ROS_HOSTNAME..."
export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
export DUCKIETOWN_ROOT=$HOME/duckietown

echo "Setting up PYTHONPATH..."
export PYTHONPATH=$DUCKIETOWN_ROOT/catkin_ws/src:$PYTHONPATH

# Cannot make machines before building
# echo "Building machines file..."
# make -C $DUCKIETOWN_ROOT machines

echo "Activating development environment..."
source $DUCKIETOWN_ROOT/catkin_ws/devel/setup.bash

# TODO: check that the time is >= 2015


exec "$@" #Passes arguments. Need this for ROS remote launching to work.
