#!/usr/bin/env python2

# To be run in the background at startup, 
# constantly listening for the joystick's start button.
# Author: garcia-mallen@duckietown.com

# My original idea was to run jstest in a shell script loop. 
# While not a good idea, it might still perform faster than this.

# future fixes and potenial modifications: 
#   - have it actually roslaunch the joystick demo
#   - multiple presses tend to be detected when the user means to send only one. 
#     Only run the shell script if it isn't running already. 
#     they haven't ben run already
#   - kill all ros processes if back and start buttons pressed simultaneously
#   - run one of several programs based on which buttons are pressed 
#     immediately after start, or at the same time as start.
#   - blink lights to respond to controller imput

# must do: 
#   - remove logging. 
#     I really dont' feel this is the way things should be logged outside of 
#     debugging. Logging should happen, but someone else should figure out
#     how to properly do it either within ROS or linux. 

import struct, subprocess
from js_linux import button_map, button_states, jsdev

if __name__ == '__main__':

    # log that this program has been started. Yes, I know I shouldn't do this in bash.
    print('joystick-daemon.py is running and listening for inputs!')

    # Main event loop. Most of this was modified from js_linux.py.

    while True:
        event_buffer = jsdev.read(8)
        if event_buffer:
            time, value, type, number = struct.unpack('IhBB', event_buffer)
            
        # If the event type is a button press
        if type & 0x01:
            button = button_map[number]
            if button:
                button_states[button] = value
                
                # Events could be either buttons being released or pressed. 
                # We're only concerned with the latter; 
                # if the start button was released; 
                if value and button == 'start':
                    # run a simple shell script.
                    subprocess.call(
                        ['/home/ubuntu/.duckietown/joystick-daemon/start-response.sh'])
