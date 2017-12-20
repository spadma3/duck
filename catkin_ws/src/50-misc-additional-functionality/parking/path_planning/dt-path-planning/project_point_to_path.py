#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Path planning for duckietown parking
Project point to given path
Samuel Nyffenegger
"""

from parking_main import *
import dubins_path_planning as dpp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from math import sin, cos, sqrt, atan2, degrees, radians, pi
from numpy import sign
import os

"""
Global parameters
"""
# control parameters
choose_random_parking_space_combination = True

# projection parameters
bias_xy = 0             # mm
var_xy = 100            # mm
bias_heading = 0;       # rad
var_heading = pi/8      # rad

# calculate controller values: d_est, d_ref, theta_est, c_ref, v_ref
# d_est, d_ref, theta_est, c_ref, v_ref = fnc(px, py, pyaw, x_act, y_act, yaw_act, curvature)

"""
Functions
"""
def get_random_pose(px, py, pyaw):
    n_points = len(px)
    idx = np.random.random_integers(0, n_points)
    x_act = px[idx] + np.random.normal(bias_xy,var_xy)
    y_act = py[idx] + np.random.normal(bias_xy,var_xy)
    yaw_act = pyaw[idx] + np.random.normal(bias_heading,var_heading)
    return x_act, y_act, yaw_act


"""
main file
"""
if __name__ == '__main__':
    print('Path planning and projection for duckietown...')

    # path planning
    px, py, pyaw = path_planning(0,3)

    # get some random pose near path
    x_act, y_act, yaw_act = get_random_pose(px, py, pyaw)
