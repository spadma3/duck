#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Path planning for duckietown parking_space

Samuel Nyffenegger
"""

import dubins_path_planning as dpp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from math import sin, cos, sqrt, atan2, degrees, radians, pi
from numpy import sign


"""
Global parameters
"""
# control parameters
do_talking = True
do_ploting = True

# parking lot parameters
lot_width = 2*585                   # mm, lot = 2x2 squares
lot_height = 2*585                  # mm
wide_tape_width = 50                # mm, red, white
narrow_tape_width = 25              # mm, yellow
april_tag_basement_length = 50      # mm,
april_tag_screen_length = 80        # mm
space_length = 270                  # mm from border, without april tag
lanes_length = 310                  # mm at entrance, exit

# path planning parameters
primitive_backwards_start = True    # drive backwards and plan afterwards
curvature = 200 #60                      # mm minimal turning radius
allow_backwards_on_circle = False   # use this later together with reeds sheep
length_red_line = int( (lot_width/2.0 - 2.0*wide_tape_width - 1.0*narrow_tape_width) / 2.0 )

# plotting parameters
visual_boundairy = 100              # mm

"""
Functions
"""
# pose assigenment: entrance, parking space, exit
def pose_from_key(key):
    if key == "entrance" or key == 0:
        return np.array([wide_tape_width+length_red_line/2.0, lot_height-lanes_length/2.0, -pi/2.0])
    elif key == "space 1" or key == 1:
        return np.array([lot_width/8.0, space_length/2.0, -pi/2.0])
    elif key == "space 2" or key == 2:
        return np.array([lot_width/8.0 + lot_width/4.0, space_length/2.0, -pi/2.0])
    elif key == "space 3" or key == 3:
        return np.array([lot_width/8.0 + 2*lot_width/4.0, space_length/2.0, -pi/2.0])
    elif key == "space 4" or key == 4:
        return np.array([lot_width/8.0 + 3*lot_width/4.0, space_length/2.0, -pi/2.0])
    elif key == "space 5" or key == 5:
        return np.array([lot_width/8.0 + 2*lot_width/4.0, lot_height-space_length/2.0, pi/2.0])
    elif key == "space 6" or key == 6:
        return np.array([lot_width/8.0 + 3*lot_width/4.0, lot_height-space_length/2.0, pi/2.0])
    elif key == "exit" or key == 7:
        return np.array([wide_tape_width+narrow_tape_width+3.0/2.0*length_red_line, lot_height-lanes_length/2.0, pi/2.0])
    else:
        print("parking space not found")
        exit(1)


"""
main file
"""
if __name__ == '__main__':
    print('Path planning for duckietown...')

    """
    problem definition
    """
    # start and endpose
    start_x, start_y, start_yaw = pose_from_key("entrance")
    end_x, end_y, end_yaw = pose_from_key("space 3")
    if do_talking:
        print("initial pose: \n\tx = {}\n\ty = {} \n\ttheta = {}".format(
        start_x, start_y, degrees(start_yaw) ))
        print("end pose: \n\tx = {}\n\ty = {} \n\ttheta = {}".format(
        end_x, end_y, degrees(end_yaw) ))
        print("curvature = {}".format(curvature))


    """
    path calculation
    """
    px, py, pyaw, mode, clen = dpp.dubins_path_planning(start_x, start_y, start_yaw,
                    end_x, end_y, end_yaw, curvature, allow_backwards_on_circle)


    """
    plot results
    """
    if do_ploting:
        fig, ax = plt.subplots(1)
        plt.plot(px, py)
        # plt.plot(px, py, label="final course " + "".join(mode))
        dpp.plot_arrow(start_x, start_y, start_yaw, 0.1*lot_width, 0.06*lot_width, fc="r", ec="r")
        dpp.plot_arrow(end_x, end_y, end_yaw, 0.1*lot_width, 0.06*lot_width, fc="g", ec="g")

        ax.add_patch( patches.Rectangle( (0.0, 0.0), lot_width, lot_height, fill=False ))
        # plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.xlim([-visual_boundairy,lot_height+visual_boundairy])
        plt.ylim([-visual_boundairy,lot_width+visual_boundairy])
        plt.show()
