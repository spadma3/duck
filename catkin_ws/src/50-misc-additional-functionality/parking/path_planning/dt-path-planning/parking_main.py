#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Path planning for duckietown parking_space

Samuel Nyffenegger
"""

from dubins_path_planning import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from math import sin, cos, sqrt, atan2, degrees, radians, pi
from numpy import sign





if __name__ == '__main__':
    print('Path planning for duckietown...')

    # problem definition
    parking_space = 4
    start_x = (parking_space-1.0)/4.0+1.0/8.0
    start_y = 0.5
    start_yaw = math.radians(-90)
    end_x = 0.2
    end_y = 1.0
    end_yaw = math.radians(90.0)
    curvature = 0.15
    allow_backwards_on_circle = False


    # path calculation
    px, py, pyaw, mode, clen = dubins_path_planning(start_x, start_y, start_yaw,
                    end_x, end_y, end_yaw, curvature, allow_backwards_on_circle)


    # plot results
    fig, ax = plt.subplots(1)
    plt.plot(px, py, label="final course " + "".join(mode))
    plot_arrow(start_x, start_y, start_yaw, 0.1, 0.06, fc="r", ec="r")
    plot_arrow(end_x, end_y, end_yaw, 0.1, 0.06, fc="g", ec="g")
    ax.add_patch( patches.Rectangle( (0.0, 0.0), 1.0, 1.0, fill=False ))
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.xlim([-0.5,1.5])
    plt.ylim([-0.5,1.5])
    plt.show()
