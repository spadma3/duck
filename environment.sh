#!/bin/bash
# New policy: DUCKIETOWN_ROOT is implicit in the choice of running this script

[ -z "$HOSTNAME"        ] && { echo -e "\n\nThe variable HOSTNAME is not set. I need this info for setting up ROS. \n\n\n\n"; return 2;       }

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/kinetic/setup.$shell

export DUCKIETOWN_ROOT=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
echo "Set DUCKIETOWN_ROOT to: $DUCKIETOWN_ROOT"

export PYTHONPATH=$DUCKIETOWN_ROOT/catkin_ws/src:$PYTHONPATH
echo "Set PYTHONPATH to: $PYTHONPATH"

# Cannot make machines before building
# echo "Building machines file..."
# make -C $DUCKIETOWN_ROOT machines

echo "Activating development environment..."
source $DUCKIETOWN_ROOT/catkin_ws/devel/setup.$shell

if [ 2015 -ge $(date +%Y) ];
then
    >&2 echo "Error! Time travel detected. System time is: $(date)"
fi

# add the shortcuts to the path
export PATH=$PATH:$DUCKIETOWN_ROOT/shortcuts

export DISABLE_CONTRACTS=1

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
