# VirtualJoy for Duckiebots

This is a virtual joystick controlled from your laptop using your arrow keys. Especially useful in the following situations:

- Large events where a lot of people use non-bounded controllers
- If you forgot to bring your controller to a meeting


<img src="https://github.com/duckietown/Software/blob/jukindle-devel-virtualjoy/misc/virtualJoy/images/d-pad.png" width="120" height="120"/>

# Usage

Start any demo on your Duckiebot, for example

    make demo-joystick
        
All the following is done **on your laptop** - this script is using pygame so you need a screen (SSH will not work)

Start from your duckietown folder
    
    source environment.sh
    source set_ros_master.sh <vehicle_name>
    source set_vehicle_name.sh <vehicle_name>
    
    cd misc/virtualJoy
    python virtualJoy.py
    
A window will open - to steer your bot, your window focus must be inside that window. 


# Commands

The following keys are supported:

| KEYS       | FUNCTION                             |
|------------|--------------------------------------|
| ARROW_KEYS | Steer your Duckiebot                 |
| q          | Quit the program                     |
| a          | Turn on line-following aka autopilot |
| s          | Stop line-following                  |
| i          | Toggle Anti-instagram                |

# FAQ

- No module named pygame

        pip install pygame
        
If you have any questions, feel free to contact Julien Kindle: jkindle@ethz.ch
