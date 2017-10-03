#!/bin/bash
[ -z "$DUCKIETOWN_ROOT" ] && { echo "Need to set DUCKIETOWN_ROOT - configuration is invalid (!)";  }
[ -z "$HOSTNAME"        ] && { echo "Need to set HOSTNAME.";        }

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

shell=`basename $SHELL`
echo "Activating ROS..."
source /opt/ros/kinetic/setup.$shell

echo "Setup ROS_HOSTNAME..."
export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
export DUCKIETOWN_ROOT=$(pwd)

echo "Setting up PYTHONPATH..."
export PYTHONPATH=$DUCKIETOWN_ROOT/catkin_ws/src:$PYTHONPATH

# Cannot make machines before building
# echo "Building machines file..."
# make -C $DUCKIETOWN_ROOT machines

echo "Activating development environment..."
source $DUCKIETOWN_ROOT/catkin_ws/devel/setup.$shell

# TODO: check that the time is >= 2015


exec "$@" #Passes arguments. Need this for ROS remote launching to work.
