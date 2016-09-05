#!/usr/bin/env bash

printf "\nInstalling joystickd: duckietown's joystick daemon\n"

# This is **NOT** the correct means of installing anything. Ideally,
# in catkin, one would add_custom_target() in CMakeLists.txt to move
# these files to their homes, and setup.py's distutils module to make
# sure supervisord is on the system via from PyPI. But this works for
# joystickd.

while true; do
    read -p "Do you wish to uninstall joystickd and supervisord? (y/n) " yn
    case $yn in
        [Yy]* ) break;;
        [Nn]* ) printf "\njoystickd uninstaller is exiting!";
	    exit;;
        * ) echo "Please answer yes or no.";;
    esac
done


# The process control manager to manage joystick-dememon.py
sudo pip install supervisor


### Copy the configuration files from the repo to where they belong. 

# this is to be run on duckiebots. $DUCKIETOWN_ROOT/environment.sh
# already assumes that $DUCKIETOWN_ROOT=$HOME/duckietown.
# While this assumption has caused me much pain, I'll assume it here.
export F11_INSTALL_DIR=$HOME/duckietown/catkin_ws/src/f11-no_laptop/install-files

# if the directory doesn't exist,
if [ ! -d "/etc/supervisor" ]; then
    sudo mkdir /etc/supervisor
fi

sudo cp $F11_INSTALL_DIR/supervisord.conf /etc/supervisor
sudo cp $F11_INSTALL_DIR/supervisor /etc/init.d

# Update the init system telling it wen supervisor should be executed and terminated.
sudo update-rc.d supervisor start 30 2 3 4 5 . stop 70 0 1 6 .

printf "\njoystickd installer is exiting!"

exit 0
