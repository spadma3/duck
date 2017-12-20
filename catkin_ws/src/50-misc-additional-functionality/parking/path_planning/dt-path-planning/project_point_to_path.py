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
from numpy import sign, mean
import os

"""
Global parameters
"""
# control parameters
choose_random_parking_space_combination = True
close_itself = True
save_figures = False
pause_per_path = 1.0 # sec
ploting = True

# projection parameters
velocity = 0.1 # m/s
bias_xy = 0             # mm
var_xy = 50            # mm
bias_heading = 0;       # rad
var_heading = pi/20      # rad

"""
Functions
"""
def get_random_pose(px, py, pyaw):
    n_points = len(px)
    idx = np.random.random_integers(1, n_points-1)
    x_act = px[idx] + np.random.normal(bias_xy,var_xy)
    y_act = py[idx] + np.random.normal(bias_xy,var_xy)
    yaw_act = pyaw[idx] + np.random.normal(bias_heading,var_heading)
    return x_act, y_act, yaw_act

def path_planning(start_number=None, end_number=None):
    # define problem
    start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number = initialize(start_number, end_number)
    obstacles = define_obstacles()

    # path planning and collision check with dubins path
    px, py, pyaw = dubins_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, obstacles)
    found_path = collision_check(px, py, obstacles, start_number, end_number)

    # get some random pose near path
    x_act, y_act, yaw_act = get_random_pose(px, py, pyaw)

    # calculate controller values: d_est, d_ref, theta_est, c_ref, v_ref
    d_est, d_ref, theta_est, c_ref, v_ref, x_proj, y_proj = project_to_path(px, py, pyaw, x_act, y_act, yaw_act, curvature)


    # show results
    # do_talking(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number)
    if ploting:
        do_plotting_projection(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number, px, py, obstacles, found_path, x_act, y_act, yaw_act, x_proj, y_proj)

    return px, py, pyaw

def project_to_path(px, py, pyaw, x_act, y_act, yaw_act, curvature):
    """
    Input:
        px, py in mm
        pyaw in radians
        x_act, y_act in mm
        yaw_act in radians
        curvature is the minimum turning radius in mm
    Output:
        d_est in mm
        d_ref in mm
        theta_est in radians, heading robot more left than path heading: negative
        c_ref in mm
    """

    # distance calculation
    d_est = float("inf")
    x_proj, y_proj, idx_proj = None, None, None
    for idx, (x, y) in enumerate(zip(px, py)):
        d = sqrt((x-x_act)**2 + (y-y_act)**2)
        if d < d_est:
            d_est = d
            x_proj, y_proj, idx_proj = x, y, idx


    # curvature or straight
    if (np.array([px[idx_proj-1],px[idx_proj+1]]).mean() == px[idx_proj]) and (np.array([py[idx_proj-1],py[idx_proj+1]]).mean() == py[idx_proj]):
        c_ref = float("inf")
    else:
        c_ref = 1.0/curvature

    # differential var_heading
    theta_est = pyaw[idx_proj] - yaw_act

    # further parameters
    d_ref, v_ref = 0, velocity

    print("d_est = {}\ntheta_est_deg = {}\nc_ref = {}".format(d_est, degrees(theta_est), c_ref))

    return d_est, d_ref, theta_est, c_ref, v_ref, x_proj, y_proj


def do_plotting_projection(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number, px, py, obstacles, found_path, x_act, y_act, yaw_act, x_proj, y_proj):
    if close_itself:
        plt.clf()
    fig, ax = plt.subplots()
    if found_path:
        plt.plot(px, py,'g-',lw=3)
    else:
        plt.plot(px, py,'m-',lw=3)

    # plt.plot(px, py, label="final course " + "".join(mode))
    dpp.plot_arrow(start_x, start_y, start_yaw,
    0.11*lot_width, 0.06*lot_width, fc="r", ec="r")
    dpp.plot_arrow(end_x, end_y, end_yaw,
    0.11*lot_width, 0.06*lot_width, fc="g", ec="g")
    ax.add_patch( patches.Rectangle( (0.0, 0.0),
    lot_width, lot_height, fill=False ))
    # plt.legend()
    plt.axis("equal")
    plt.xlim([-visual_boundairy,lot_height+visual_boundairy])
    plt.ylim([-visual_boundairy,lot_width+visual_boundairy])
    for obstacle in obstacles:
        if obstacle[5]:
            ax.add_patch( patches.Rectangle( (obstacle[0], obstacle[1]),
            obstacle[2], obstacle[3], fc=obstacle[4] ))
        else:
            ax.add_patch( patches.Rectangle( (obstacle[0], obstacle[1]),
            obstacle[2], obstacle[3], fc=obstacle[4], ec="m", hatch='x'))

    dpp.plot_arrow(x_act, y_act, yaw_act,
    0.11*lot_width, 0.06*lot_width, fc="c", ec="c")

    plt.plot([x_proj, x_act],[y_proj, y_act],c='c')

    if close_itself:
        plt.draw()
        plt.pause(pause_per_path)
    else:
        plt.show()

    if save_figures:
        dic = {True:'driveable', False:'collision'}
        plt.savefig('images/path_{}_{}_{}.pdf'.format(start_number,end_number,dic[found_path]))


"""
main file
"""
if __name__ == '__main__':
    print('Path planning and projection for duckietown...')

    path_planning(0,3)
