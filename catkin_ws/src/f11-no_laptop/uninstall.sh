#!/usr/bin/env bash

printf "\n\nUninstalling joystickd: duckietown's joystick daemon\n"


# supervisor may have been installed separately from joystickd, 
# so a warning is warranted. See ./install.sh for more information 
while true; do
    read -p "Do you wish to uninstall supervisord? (y/n) " yn
    case $yn in
        [Yy]* ) sudo pip uninstall supervisor; 
	    sudo rm -r /etc/supervisor;
	    sudo rm supervisor /etc/init.d/supervisor;
	    sudo update-rc.d supervisor remove;
	    break;;
        [Nn]* ) printf "\njoystickd uninstaller is exiting!";
	    exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

exit 0;

