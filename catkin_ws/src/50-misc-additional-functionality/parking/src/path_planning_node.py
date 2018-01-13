#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import dubins_path_planning as dpp
import numpy as np
from math import sin, cos, sqrt, atan2, degrees, radians, pi
from duckietown_msgs.msg import Pose2DStamped, LanePose  # custom message to subscribe to

"""
Global parameters
"""

# path planning parameters
radius_robot = 70                   # mm distance point between wheels and most apart point on robot
straight_in_parking_space = True    # robot drives last forward bit straigt (robustness increase)
straight_at_entrance = True         # robot drives last forward bit straigt (robustness increase)
primitive_backwards = True          # drive backwards and plan afterwards
allow_backwards_on_circle = False   # use this later together with reeds sheep
curvature = 120                     # mm minimal turning radius
n_nodes_primitive = 50              # -
distance_backwards = 400            # mm

# parking lot parameters
lot_width = 2*585                   # mm, lot = 2x2 squares
lot_height = 2*585                  # mm
wide_tape_width = 50                # mm, red, white
narrow_tape_width = 25              # mm, yellow
april_tag_basement_length = 50      # mm,
april_tag_screen_length = 80        # mm
space_length = 270                  # mm from border, without april tag
lanes_length = 310                  # mm at entrance, exit



# additional constants
length_red_line = (lot_width/2.0 - 2.0*wide_tape_width - 1.0*narrow_tape_width) / 2.0

class parkingPathPlanner():

    def __init__(self):
        self.plan = True
        self.sample_freq = 50
        self.d_ref = 0  # for parking, d_ref = 0
        self.v_ref = 0.05  # reference vel for parking
        # init counter
        #self.count = 0
        # init subscriber
        rospy.Subscriber("~pose_duckiebot", Pose2DStamped, self.localization_callback)
        # init pose
        #pose =  Pose2DStamped()
        #self.x_act = 0 #165.0
        #self.y_act = 0 #1015
        #self.yaw_act = 0 #-pi/2
        #print "The pose is initialized to: ",(self.x_act,self.y_act,self.yaw_act)
        # init publisher
        self.sample_state_pub = rospy.Publisher("~parking_pose", LanePose, queue_size=1)
        #self.path_planning(rospy.get_param('~end_space'))
        #print "The computed x path is ", self.px
        #print "The computed y path is ", self.py
        #print "The computed yaw path is ", self.pyaw
        self.timer = rospy.Timer(rospy.Duration(1.0/self.sample_freq), self.sample_callback)

    #  callback for control references
    def sample_callback(self,event):
        state = LanePose()
        if self.plan == False:
            state.d, state.curvature, state.phi = self.project_to_path(curvature)
            state.d_ref = self.d_ref
            state.v_ref = self.v_ref
            self.sample_state_pub.publish(state)

    #  callback for apriltag localization
    def localization_callback(self, pose):
        if self.plan == True:    
            # plan the path once during first callback
            self.x_act = pose.x
            self.y_act = pose.y
            self.yaw_act = pose.theta
            self.path_planning(rospy.get_param('~end_space'))
            print "The pose is initialized to: ",(self.x_act,self.y_act,self.yaw_act)
            self.plan = False
        else:
            self.x_act = pose.x
            self.y_act = pose.y
            self.yaw_act = pose.theta


        return self.x_act, self.y_act, self.yaw_act

    def project_to_path(self, curvature):
        """
        Input:
            curvature [mm] minimum turning radius
        Output:
            d_est [m] d_est > d_ref for vehicle left of reference
            theta_est [radians] Anti-clockwise
            c_ref in [1/m]
        """
        # distance calculation
        d_est = float("inf")
        x_proj, y_proj, idx_proj = None, None, None
        for idx, (x, y, yaw) in enumerate(zip(self.px, self.py, self.pyaw)):
            d = sqrt((x-self.x_act)**2 + (y-self.y_act)**2)
            if d < abs(d_est):
                d_est = d
                x_proj, y_proj, yaw_proj, idx_proj = x, y, yaw, idx
        # A:projected point and frame with origin A and “heading“ yaw_proj, I:inertial frame
        R_AI = np.array([[cos(yaw_proj),sin(yaw_proj)],[-sin(yaw_proj),cos(yaw_proj)]])
        I_t_IA = np.array([[x_proj],[y_proj]])
        A_t_AI = np.dot(-R_AI,I_t_IA)
        p_act_A = np.dot(R_AI,np.array([[self.x_act],[self.y_act]]))+A_t_AI
        if p_act_A[1] < 0:
            d_est = -d_est

        # curvature or straight (only valid for dubins path)
        if ((self.px[idx_proj-1]+self.px[idx_proj+1])/2.0 == self.px[idx_proj]) and ((self.py[idx_proj-1]+self.py[idx_proj+1])/2.0 == self.py[idx_proj]):
            c_ref = 0.0
        else:
            c_ref = 1.0/curvature

        # differential var_heading
        theta_est = self.yaw_act - self.pyaw[idx_proj]

        # further parameters
        #d_ref, v_ref = 0

        #print("d_est = {}\ntheta_est_deg = {}\nc_ref = {}".format(d_est/1000.0, degrees(theta_est), c_ref/1000.0))

        return d_est/1000.0, c_ref*1000.0, theta_est


    def path_planning(self, end_number):
        # define problem
        end_x, end_y, end_yaw = self.initialize(end_number)
        start_x   = self.x_act
        start_y   = self.y_act
        start_yaw = self.yaw_act
        objects = self.define_objects()
        obstacles = self.define_obstacles(objects)

        # path planning and collision check with dubins path
        self.px, self.py, self.pyaw = self.dubins_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw)
        self.collision_check(self.px, self.py, obstacles)

    def collision_check(self,px, py, obstacles):
        found_path = True
        crash, out_of_parking_lot = False, False
        for x, y in zip(px, py):
            for obstacle in obstacles:
                if (x <= 0.0 or lot_width <= x or y <= 0.0 or lot_height <= y):
                    found_path = False
                    out_of_parking_lot = True
                if obstacle[0] == "rectangle":
                    if (obstacle[1] < x and x < obstacle[1] + obstacle[3]) and (
                            obstacle[2] < y and y < obstacle[2] + obstacle[4]):
                        found_path = False
                        crash = True
                elif obstacle[0] == "circle":
                    if (sqrt((x - obstacle[1]) ** 2 + (y - obstacle[2]) ** 2) < obstacle[3]):
                        found_path = False
                        crash = True
                else:
                    exit("SN:ERROR: type {} not known.".format(obstacle[0]))

        if found_path:
            print("A collision free path was found!")
        else:
            print("No collision free path was found!")
            if crash:
                print("\tThe robot will crash into objects on this path!")
            if out_of_parking_lot:
                print("\tThe robot wants to drive outside the parking lot")

    # init for every new path
    def initialize(self, end_number):

        end_x, end_y, end_yaw = self.pose_from_key(end_number)

        return end_x, end_y, end_yaw

    # pose assigenment: entrance, parking space, exit
    def pose_from_key(self, key):
        if key == "entrance" or key == 0:
            return np.array([wide_tape_width + length_red_line / 2.0,
                             lot_height - lanes_length / 4.0 * 2.0, -pi / 2.0])
        elif key == "space 1" or key == 1:
            return np.array([lot_width / 8.0, space_length / 2.0, -pi / 2.0])
        elif key == "space 2" or key == 2:
            return np.array([lot_width / 8.0 + lot_width / 4.0,
                             space_length / 2.0, -pi / 2.0])
        elif key == "space 3" or key == 3:
            return np.array([lot_width / 8.0 + 2 * lot_width / 4.0,
                             space_length / 2.0, -pi / 2.0])
        elif key == "space 4" or key == 4:
            return np.array([lot_width / 8.0 + 3 * lot_width / 4.0,
                             space_length / 2.0, -pi / 2.0])
        elif key == "space 5" or key == 5:
            return np.array([lot_width / 8.0 + 2 * lot_width / 4.0,
                             lot_height - space_length / 2.0, pi / 2.0])
        elif key == "space 6" or key == 6:
            return np.array([lot_width / 8.0 + 3 * lot_width / 4.0,
                             lot_height - space_length / 2.0, pi / 2.0])
        elif key == "exit" or key == 7:
            return np.array([wide_tape_width + narrow_tape_width + 3.0 / 2.0 * length_red_line,
                             lot_height - lanes_length / 4.0 * 2.0, pi / 2.0])
        elif key == "watch" or key == 8:
            return np.array([lot_width / 2.0, lot_height / 2.0, 0.0])
        else:
            print("parking space '{}' not found".format(key))
            exit(1)

    # define objects and obstacles
    def define_objects(self):
        # x, y, dx, dy, colour, driveable
        objects = []
        objects.append((0.0, 0.0, narrow_tape_width, space_length, "b", True))
        objects.append((lot_width / 4.0 - narrow_tape_width / 2.0, 0.0, narrow_tape_width, space_length, "b", True))
        objects.append((lot_width / 2.0 - narrow_tape_width / 2.0, 0.0, narrow_tape_width, space_length, "b", True))
        objects.append(
            (lot_width / 4.0 * 3.0 - narrow_tape_width / 2.0, 0.0, narrow_tape_width, space_length, "b", True))
        objects.append((lot_width - narrow_tape_width, 0.0, narrow_tape_width, space_length, "b", True))
        objects.append((lot_width / 4.0 * 3.0 - narrow_tape_width / 2.0, lot_height - space_length, narrow_tape_width,
                        space_length, "b", True))
        objects.append(
            (wide_tape_width + length_red_line, lot_height - lanes_length, narrow_tape_width, lanes_length, "y", True))
        objects.append((0.0, lot_height - lanes_length, wide_tape_width, lanes_length, "w", True))
        objects.append((
                       lot_width / 2.0 - wide_tape_width, lot_height - lanes_length, wide_tape_width, lanes_length, "w",
                       False))  # False
        objects.append((wide_tape_width, lot_height - lanes_length, length_red_line, wide_tape_width, "r", True))
        objects.append((wide_tape_width + narrow_tape_width + length_red_line, lot_height - wide_tape_width,
                        length_red_line, wide_tape_width, "r", True))
        # objects.append((wide_tape_width+narrow_tape_width+length_red_line, lot_height-lanes_length,length_red_line, wide_tape_width, "m", False))

        return objects

    def define_obstacles(self,objects):
        # rectangle, x, y, dx, dy
        #  circle, x, y, r
        obstacles = []
        for obj in objects:
            if not obj[5]:  # object not driveable
                # current implementation only valid if object is a rectangle
                obstacles.append(("rectangle", obj[0] - radius_robot, obj[1], obj[2] + 2.0 * radius_robot, obj[3]))
                obstacles.append(("rectangle", obj[0], obj[1] - radius_robot, obj[2], obj[3] + 2.0 * radius_robot))
                obstacles.append(("circle", obj[0], obj[1], radius_robot))
                obstacles.append(("circle", obj[0] + obj[2], obj[1], radius_robot))
                obstacles.append(("circle", obj[0], obj[1] + obj[3], radius_robot))
                obstacles.append(("circle", obj[0] + obj[2], obj[1] + obj[3], radius_robot))

        return obstacles

    #  dubins path planning
    def dubins_path_planning(self, start_x, start_y, start_yaw, end_x, end_y, end_yaw):
        # heuristics using path primitives
        detect_space_14 = (start_y < space_length and (abs(start_yaw + radians(90)) < radians(45)))
        detect_space_56 = lot_height - start_y < space_length and abs(start_yaw - radians(90)) < radians(
            45) and lot_width / 2.0 < start_x
        if primitive_backwards and (detect_space_14 or detect_space_56):
            dt = distance_backwards / n_nodes_primitive
            px_backwards = [start_x]
            py_backwards = [start_y]
            pyaw_backwards = [start_yaw]
            for i in range(n_nodes_primitive):
                px_backwards.append(px_backwards[-1] - dt * cos(pyaw_backwards[-1]))
                py_backwards.append(py_backwards[-1] - dt * sin(pyaw_backwards[-1]))
                pyaw_backwards.append(pyaw_backwards[-1])
            start_x = px_backwards[-1]
            start_y = py_backwards[-1]
            start_yaw = pyaw_backwards[-1]

        start_x_0, start_y_0, start_yaw_0 = self.pose_from_key(0)
        straight_at_entrance_ = (
        straight_at_entrance and abs(start_x - start_x_0) < 1.0 and abs(start_y - start_y_0) < 1.0 and abs(
            start_yaw - start_yaw_0) < 1.0)
        if straight_at_entrance_:
            dt = space_length / 2.0 / n_nodes_primitive
            px_straight_entrance = [start_x]
            py_straight_entrance = [start_y]
            pyaw_straight_entrance = [start_yaw]
            for i in range(n_nodes_primitive):
                px_straight_entrance.append(px_straight_entrance[-1] + dt * cos(pyaw_straight_entrance[-1]))
                py_straight_entrance.append(py_straight_entrance[-1] + dt * sin(pyaw_straight_entrance[-1]))
                pyaw_straight_entrance.append(pyaw_straight_entrance[-1])
            start_x = px_straight_entrance[-1]
            start_y = py_straight_entrance[-1]
            start_yaw = pyaw_straight_entrance[-1]

        if straight_in_parking_space:
            dt = space_length / 2.0 / n_nodes_primitive
            px_straight = [end_x]
            py_straight = [end_y]
            pyaw_straight = [end_yaw]
            for i in range(n_nodes_primitive):
                px_straight.append(px_straight[-1] - dt * cos(pyaw_straight[-1]))
                py_straight.append(py_straight[-1] - dt * sin(pyaw_straight[-1]))
                pyaw_straight.append(pyaw_straight[-1])
            end_x = px_straight[-1]
            end_y = py_straight[-1]
            end_yaw = pyaw_straight[-1]
            px_straight.reverse()
            py_straight.reverse()
            pyaw_straight.reverse()

        # actual path plannign using dubin curves
        px, py, pyaw, mode, clen = dpp.dubins_path_planning(start_x,
                                                            start_y, start_yaw, end_x, end_y, end_yaw, curvature,
                                                            allow_backwards_on_circle)

        #  add path primitives to path
        if primitive_backwards and (detect_space_14 or detect_space_56):
            px = px_backwards + px
            py = py_backwards + py
            pyaw = pyaw_backwards + pyaw

        if straight_at_entrance_:
            px = px_straight_entrance + px
            py = py_straight_entrance + py
            pyaw = pyaw_straight_entrance + pyaw

        if straight_in_parking_space:
            px = px + px_straight
            py = py + py_straight
            pyaw = pyaw + pyaw_straight

        return px, py, pyaw


"""
main file
"""
if __name__ == '__main__':
    print('Path planning and projection for duckietown...')
    rospy.init_node('park_path_planning_node')
    pPP = parkingPathPlanner()
    rospy.spin()

    #path_planning(0,3)


