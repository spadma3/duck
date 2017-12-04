#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
if 'python path_plannning_v1.py' works, but './path_planning_v1.py' does not, use this in terminal:
sed -i -e 's/\r$//' scriptname.sh
"""

"""
Functions
"""
# helper function to draw an arrow in the map
def plot_pose_as_arrow(pose, colour="k", arrow_magnitude=75):
    # this function swaps x,y in pose to generate the arrow
    plt.arrow( pose[1], pose[0], arrow_magnitude*np.cos(-pose[2]), arrow_magnitude*np.sin(-pose[2]), fc=colour, ec=colour, head_width=arrow_magnitude/5.0, head_length=arrow_magnitude/5.0)

# pose assigenment: entrance, parking space, exit
def pose_from_key(key):
    if key == "entrance" or key == 0:
        return np.array([lanes_length/2.0, wide_tape_width+length_red_line/2.0, -math.pi/2.0])
    elif key == "space 1" or key == 1:
        return np.array([lot_height-space_length/2.0, lot_width/8.0, -math.pi/2.0])
    elif key == "space 2" or key == 2:
        return np.array([lot_height-space_length/2.0, lot_width/8.0 + lot_width/4.0, -math.pi/2.0])
    elif key == "space 3" or key == 3:
        return np.array([lot_height-space_length/2.0, lot_width/8.0 + 2*lot_width/4.0, -math.pi/2.0])
    elif key == "space 4" or key == 4:
        return np.array([lot_height-space_length/2.0, lot_width/8.0 + 3*lot_width/4.0, -math.pi/2.0])
    elif key == "space 5" or key == 5:
        return np.array([space_length/2.0, lot_width/8.0 + 2*lot_width/4.0, math.pi/2.0])
    elif key == "space 6" or key == 6:
        return np.array([space_length/2.0, lot_width/8.0 + 3*lot_width/4.0, math.pi/2.0])
    elif key == "exit" or key == 7:
        return np.array([lanes_length/2.0, wide_tape_width+narrow_tape_width+3.0/2.0*length_red_line, math.pi/2.0])
    else:
        print("parking space not found")
        exit(1)



"""
Initialize
"""
import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
args = sys.argv


"""
Static map generation
"""
### control parameters
plot_static_map = True

### parking lot parameters
lot_width = 2*585               # mm, lot = 2x2 squares
lot_height = 2*585              # mm
wide_tape_width = 50            # mm, red, white
narrow_tape_width = 25          # mm, yellow
april_tag_basement_length = 50  # mm,
april_tag_screen_length = 80    # mm
space_length = 270              # mm from border, without april tag
lanes_length = 310               # mm at entrance, exit

### key: object on map <--> number
map_keys = {
    "empty": 0,
    "white lane": 1,
    "yellow lane": 2,
    "red lane": 3,
    "april tag": 4,
    "parked DB": 5,
}

### build map
# empty map
map = np.empty((lot_height,lot_width))

# red stopping lines
length_red_line = int( (lot_width/2.0 - 2.0*wide_tape_width - 1.0*narrow_tape_width) / 2.0 )
map[lanes_length-wide_tape_width:lanes_length,wide_tape_width:wide_tape_width+length_red_line] = map_keys["red lane"]
map[0:wide_tape_width,wide_tape_width+length_red_line+narrow_tape_width:lot_width/2.0-wide_tape_width] = map_keys["red lane"]

# white lanes
map[0:lanes_length,0:wide_tape_width] = map_keys["white lane"]
map[0:lanes_length,lot_width/2.0-wide_tape_width:lot_width/2.0] = map_keys["white lane"]

# yellow lanes
map[0:lanes_length,wide_tape_width+length_red_line:wide_tape_width+length_red_line+narrow_tape_width] = map_keys["yellow lane"]

map[lot_height-space_length:lot_height,0:narrow_tape_width] = map_keys["yellow lane"]
map[lot_height-space_length:lot_height,lot_height/4.0-narrow_tape_width/2.0:lot_height/4.0+narrow_tape_width/2.0] = map_keys["yellow lane"]
map[lot_height-space_length:lot_height,lot_height/2.0-narrow_tape_width/2.0:lot_height/2.0+narrow_tape_width/2.0] = map_keys["yellow lane"]
map[lot_height-space_length:lot_height,lot_height*3.0/4.0-narrow_tape_width/2.0:lot_height*3.0/4.0+narrow_tape_width/2.0] = map_keys["yellow lane"]
map[lot_height-space_length:lot_height,lot_width-narrow_tape_width:lot_width] = map_keys["yellow lane"]
map[0:space_length,lot_height*3.0/4.0-narrow_tape_width/2.0:lot_height*3.0/4.0+narrow_tape_width/2.0] = map_keys["yellow lane"]


"""
path planing problem
"""
# start and endpose
pose_duckiebot = pose_from_key("space 3")
pose_parking_space = pose_from_key("exit")

# 2d configuration space: (no heading, no minimum curvature condition)



"""
Plot
"""
# control param
plot_static_map = True

# param

if plot_static_map:
    # map
    plt.matshow(map)

    # arrows: pose duckiebot
    plot_pose_as_arrow(pose_duckiebot, "r")
    plot_pose_as_arrow(pose_parking_space, "g")

    plt.show()
