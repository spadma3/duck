#!/usr/bin/env bash

# This is **NOT** the correct means of installing anything.
# But it works for joystickd, and time is pressing.

# The process control manager to manage joystick-dememon.py
pip install supervisor


### Copy the configuration files from the repo to where they belong. 

# this is to be run on duckiebots. $DUCKIETOWN_ROOT/environment.sh
# already assumes that $DUCKIETOWN_ROOT=$HOME/duckietown.
# While this assumption has caused me much pain, I'll assume it here.
export F11_INSTALL_DIR=$HOME/duckietown/software/catkin_ws/src/f11-no_laptop/install-files


# if the directory doesn't exist,
if [ ! -d "/etc/supervisor" ]; then
    sudo mkdir /etc/supervisor
fi

sudo cp $F11_INSTALL_DIR/supervisor.conf /etc/init.d
sudo cp $F11_INSTALL_DIR/supervisor /etc/init.d

### Update the init system telling it wen supervisor should
#   be executed and terminated.

# the dots in this command are part of the syntax, not directories.
# see synopsis in $ man update-rc.d
sudo update-rc.d supervisor start 30 2 3 4 5 . stop 70 0 1 6 .

exit 0
