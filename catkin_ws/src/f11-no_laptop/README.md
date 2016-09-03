# f11-no_laptop/README.md

This is an interface for quickly starting programs on the duckiebot using your joystick controller. See [here](https://github.com/jogama/6.uap-paper/blob/master/jogama-6uap.pdf) for details from Sept 2016 and for work that *really should* still be done. 

## Installation
I'll try to use [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) for y'all to install this in a more 2.166 manner. This here documentation is mostly for myself, documenting as I learn. I don't know that anyone else in Duckietown has done this:

	 $ cd ~/catkin_ws
     $ catkin_make install

Which would be equivalent to calling make like this:

     $ cd ~/catkin_ws/build
     # If cmake hasn't already been called
     $ cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel
     $ make
     $ make install
	
this is not
