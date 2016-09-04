# f11-no_laptop/README.md (branch:f11-bash)

This is an interface for quickly starting programs on the duckiebot using your joystick controller. See [here](https://github.com/jogama/6.uap-paper/blob/master/jogama-6uap.pdf) for details from Sept 2016 and for further work to be done on this project.

## Installation
This a quick hack. Run 
    
	$ sudo f11-no_laptop/install.sh. 

And it's installed!

## Why is it a hack?
Because the ROS means of installing files involve invoking [catkin_make install](http://wiki.ros.org/catkin/commands/catkin_make), with our CMakeLists.txt edited correctly. This equates to the more standard

    $ cmake ..
	$ make
	$ make install
	
I do not know how much time this would take to set up, hence this stopgap. 

## How can this become less hackish?
Also written in ./install.sh:

> This is **NOT** the correct means of installing anything. Ideally,
> in catkin, one would add_custom_target() in CMakeLists.txt to move
> these files to their homes, and setup.py's distutils module to make
> sure supervisord is on the system via from PyPI. But this works for
> joystickd.
