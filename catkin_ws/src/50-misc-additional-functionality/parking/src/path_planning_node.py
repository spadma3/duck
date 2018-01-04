#!/usr/bin/env python

import rospy
from parking_main import *  # imports everything from parking_main
import dubins_path_planning as dpp
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from math import sin, cos, sqrt, atan2, degrees, radians, pi
from numpy import sign, mean
from parking.msg import Reference_for_control  # custom message to publish
from parking.msg import Pose_duckiebot  # custom message to subscribe to

"""
Global parameters
"""

class parkingPathPlanner():
    def __init__(self):
        velocity = 0.1  # m/s
        sample_freq = 50
        rospy.Subscriber("XXXXXXX", Pose_duckiebot, self.localization_callpack)
        self.sample_state_pub = rospy.Publisher('curr_sample_state', Reference_for_control)
        rospy.Timer(rospy.Duration(1/sample_freq), self.sample_callback)

    def sample_callback(self):
        state =  Reference_for_control()
        state.d = 1
        state.c = 2
        state.phi = 3
        self.sample_state_pub.publish(state)

    #  callback for apriltag localization node
    def localization_callback(pose):
        x_act = pose.x_act
        y_act = pose.y_act
        yaw_act = pose.yaw_act
        return x_act, y_act, yaw_act

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
        if ((px[idx_proj-1]+px[idx_proj+1])/2.0 == px[idx_proj]) and ((py[idx_proj-1]+py[idx_proj+1])/2.0 == py[idx_proj]):
            c_ref = 0.0
        else:
            c_ref = 1.0/curvature

        # differential var_heading
        theta_est = yaw_act - pyaw[idx_proj]

        # further parameters
        d_ref, v_ref = 0, velocity

        print("d_est = {}\ntheta_est_deg = {}\nc_ref = {}".format(d_est/1000.0, degrees(theta_est), c_ref/1000.0))

        return d_est/1000.0, d_ref/1000.0, theta_est, c_ref*1000.0, v_ref, x_proj, y_proj


    def path_planning(start_number=None, end_number=None):

        #  init ROS node, set up subscribers,publishers
        pub = rospy.Publisher('reference_for_control', Reference_for_control, queue_size=10)
        rospy.init_node('path_planning', anonymous=True)
        rospy.Subscriber('pose_duckiebot', Pose_duckiebot, localization_callback, queue_size=10)


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

"""
main file
"""
if __name__ == '__main__':
    print('Path planning and projection for duckietown...')
    rospy.init_node('parking_path_planning')

    #path_planning(0,3)


