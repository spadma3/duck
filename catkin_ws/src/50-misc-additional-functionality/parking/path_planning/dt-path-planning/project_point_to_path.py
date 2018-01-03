#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Path planning for duckietown parking
Project point to given path
Samuel Nyffenegger
"""

from parking_main import (initialize, define_objects, define_obstacles, dubins_path_planning, collision_check,
    curvature, lot_width, lot_height, visual_boundairy, radius_robot)        ### from parking_main import *
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
close_itself = False
save_figures = False
pause_per_path = 1.0 # sec
ploting = True

# projection parameters
velocity = 0.1 # m/s
bias_xy = 0.0             # mm
var_xy = 50.0            # mm
bias_heading = 0.0;       # rad
var_heading = pi/20      # rad

"""
Functions
"""
def get_random_pose(px, py, pyaw):
    n_points = len(px)
    ##### current_time_sec = rospy.time.now().secs
    ##### delta_t = current_time_sec - previous_time_sec
    idx = np.random.random_integers(1, n_points-1)          ##### idx += int(velocity * delta_t / (sqrt((px[idx] - px[idx-1])**2 + (py[idx] - py[idx-1])**2)))
    x_act = px[idx] + np.random.normal(bias_xy,var_xy)
    y_act = py[idx] + np.random.normal(bias_xy,var_xy)
    yaw_act = pyaw[idx] + np.random.normal(bias_heading,var_heading)
    ##### previous_time_sec = current_time_sec
    return x_act, y_act, yaw_act

def path_planning(start_number=None, end_number=None):
    # define problem
    start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number = initialize(start_number, end_number)
    objects = define_objects()
    obstacles = define_obstacles(objects)

    # path planning and collision check with dubins path
    px, py, pyaw = dubins_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw)
    found_path = collision_check(px, py, obstacles, start_number, end_number)

    # get some random pose near path
    x_act, y_act, yaw_act = get_random_pose(px, py, pyaw)

    # calculate controller values: d_est, d_ref, theta_est, c_ref, v_ref
    d_est, d_ref, theta_est, c_ref, v_ref, x_proj, y_proj = project_to_path(px, py, pyaw, x_act, y_act, yaw_act, curvature)

    # show results
    # do_talking(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number)
    if ploting:
        do_plotting_projection(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number, px, py, objects, obstacles, found_path, x_act, y_act, yaw_act, x_proj, y_proj  )

def project_to_path(px, py, pyaw, x_act, y_act, yaw_act, curvature):
    """
    Input:
        px, py [mm]
        pyaw [radians]
        x_act, y_act [mm]
        yaw_act [radians]
        curvature [mm] minimum turning radius
    Output:
        d_est [m] d_est > d_ref for vehicle left of reference
        d_ref [m]
        theta_est [radians] Anti-clockwise
        c_ref in [1/m]
        v_ref [m/s]
    """

    # distance calculation
    d_est = float("inf")
    x_proj, y_proj, idx_proj = None, None, None
    for idx, (x, y, yaw) in enumerate(zip(px, py, pyaw)):
        d = sqrt((x-x_act)**2 + (y-y_act)**2)
        if d < abs(d_est):
            d_est = d
            x_proj, y_proj, yaw_proj, idx_proj = x, y, yaw, idx
    # A:projected point and frame with origin A and “heading“ yaw_proj, I:inertial frame
    R_AI = np.array([[cos(yaw_proj),sin(yaw_proj)],[-sin(yaw_proj),cos(yaw_proj)]])
    I_t_IA = np.array([[x_proj],[y_proj]])
    A_t_AI = np.dot(-R_AI,I_t_IA)
    p_act_A = np.dot(R_AI,np.array([[x_act],[y_act]]))+A_t_AI
    if p_act_A[1] < 0:
        d_est = -d_est


    # curvature or straight (only valid for dubins path)
    if abs(np.cross([px[idx_proj+1] - px[idx_proj], py[idx_proj+1] - py[idx_proj], 0], [px[idx_proj+2] - px[idx_proj], py[idx_proj+2] - py[idx_proj], 0])[2]) < 1e-10:         ### if ((px[idx_proj-1]+px[idx_proj+1])/2.0 == px[idx_proj]) and ((py[idx_proj-1]+py[idx_proj+1])/2.0 == py[idx_proj]):
        c_ref = 0.0
    elif np.cross([px[idx_proj+1] - px[idx_proj], py[idx_proj+1] - py[idx_proj], 0], [px[idx_proj+2] - px[idx_proj], py[idx_proj+2] - py[idx_proj], 0])[2] < 0:
        c_ref = -1.0/curvature
    else:
        c_ref = 1.0/curvature

    # differential var_heading
    theta_est = yaw_act - pyaw[idx_proj]

    # further parameters
    d_ref, v_ref = 0, velocity

    print("d_est = {}\ntheta_est_deg = {}\nc_ref = {}".format(d_est/1000.0, degrees(theta_est), c_ref*1000.0))

    return d_est/1000.0, d_ref/1000.0, theta_est, c_ref*1000.0, v_ref, x_proj, y_proj

def do_plotting_projection(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number, px, py, objects, obstacles, found_path, x_act, y_act, yaw_act, x_proj, y_proj):
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

    # background
    ax.add_patch( patches.Rectangle( (0.0, 0.0), lot_width, lot_height, fc=(0.3,0.3,0.3)))

    # obstacles
    for obstacle in obstacles:
        if obstacle[0] == "rectangle":
            ax.add_patch( patches.Rectangle( (obstacle[1], obstacle[2]),
            obstacle[3], obstacle[4], fc="m", ec="m", hatch='x',lw=0.0))
        if obstacle[0] == "circle":
            ax.add_patch( patches.Circle( (obstacle[1], obstacle[2]),
            obstacle[3], fc="m", ec="m", hatch='x',lw=0.0))

    # boundairies
    r = 1.1*radius_robot
    ax.add_patch( patches.Rectangle( (0.0-r, 0.0-r), r, lot_height+2.0*r, fc="w", ec="w"))
    ax.add_patch( patches.Rectangle( (0.0-r, 0.0-r), lot_height+2.0*r, r, fc="w", ec="w"))
    ax.add_patch( patches.Rectangle( (0.0-r, lot_height), lot_width+2.0*r, lot_height, fc="w", ec="w"))
    ax.add_patch( patches.Rectangle( (lot_width, 0.0-r), r, lot_height+2.0*r, fc="w", ec="w"))


    for obj in objects:
        if obj[5]:
            ax.add_patch( patches.Rectangle( (obj[0], obj[1]),
            obj[2], obj[3], fc=obj[4]))
        else:
            ax.add_patch( patches.Rectangle( (obj[0], obj[1]),
            obj[2], obj[3], fc=obj[4], ec="m", hatch='x'))
    ax.add_patch( patches.Rectangle( (0.0, 0.0), lot_width, lot_height, fc=(0.3,0.3,0.3),fill=False))

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
