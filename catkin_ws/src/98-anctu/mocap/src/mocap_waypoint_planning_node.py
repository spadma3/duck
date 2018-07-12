#!/usr/bin/env python
import rospy
import roscpp
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from geometry_msgs.msg import PoseArray, Point
import tf
import sys
import time
import threading

class MocapWaypointPlanningNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # start patrolling
        self.start = True
        # waypoint position
        self.waypoint_index = 0
        self.X = [1.27, 1.27, 0.26, 0.17, 1.24, 1.27, 0.18, 0.16, 1.23]
        self.Y = [0.21, 0.51, 0.45, 0.71, 0.79, 1.04, 1.07, 1.29, 1.36]

        # vehicle point pair
        self.vehicle_yaw_pre = 0
        self.vehicle_front_point = Point()
        self.vehicle_back_point = Point()
        # the previous vehicle point of last moment
        #self.pre_vehicle_point = Point()

        # goal distance check
        self.dis_goal = 0.15

        self.kd = 0.04
        self.kp = 0.08

        self.u_pre = 0
        self.ku = 0

        self.switch = True

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        # Subscription
        self.sub_vehicle_pose_pair = rospy.Subscriber("~vehicle_pose_pair", PoseArray, self.cbPoseArray, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.onShutdown)

        # timer
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def cbPoseArray(self, point_array_msg):
        if not(self.start):
            return
        # assign vehicle point pair 
        self.vehicle_front_point = point_array_msg.poses[0].position
        self.vehicle_back_point = point_array_msg.poses[1].position
        self.vehicle_center_point = Point()
        self.vehicle_center_point.x = 0.5 * (self.vehicle_front_point.x + self.vehicle_back_point.x) 
        self.vehicle_center_point.y = 0.5 * (self.vehicle_front_point.y + self.vehicle_back_point.y) 
        self.vehicle_center_point.z = 0.5 * (self.vehicle_front_point.z + self.vehicle_back_point.z) 

        print "vehicle center: ", self.vehicle_center_point.x, self.vehicle_center_point.y, self.vehicle_center_point.z
        # set target waypoint position
        target_point = self.set_target_point(self.waypoint_index)

        # calculate yaw angle from vehicle to target waypoint
        target_yaw = self.get_yaw_two_point(self.vehicle_back_point, target_point)

        # calculate yaw angle from vehicle to previous vehicle
        vehicle_yaw = self.get_yaw_two_point(self.vehicle_back_point, self.vehicle_front_point)

        dist = self.get_dist_two_point(self.vehicle_center_point, target_point)

        #self.set_pre_vehicle_point(point_msg)
        print "yaw from vehicle to waypoint: ", target_yaw
        print "yaw from vehicle to vehicle: ", vehicle_yaw
        print "distance between vehicle and waypoint", dist

        if (dist <= self.dis_goal):
            self.waypoint_index += 1
            return
        ess = vehicle_yaw - target_yaw
        if (ess < -180):
            ess += 360
        if (ess > 180):
            ess -= 360
        diff = vehicle_yaw - self.vehicle_yaw_pre
        self.vehicle_yaw_pre = vehicle_yaw
        print "ess: ", ess
        print "diff: ", diff
        u = -1 * (self.kp * ess + self.kd * diff)
        print 'omega pd: ', u
        if( u < -7):
            u = -7
        if( u > 7):
            u = 7
        u = u * (1 - self.ku) + self.u_pre * self.ku
        print 'mega pd com: ', u
        self.publish_car_cmd(0.2, u , 0.05)
        self.u_pre = u

    def set_target_point(self, order):
        # set a target_point
        print "the ",(order+1)," point"
        target_point = Point()
        target_point.x = self.X[order]
        target_point.y = self.Y[order]
        target_point.z = 0
        return target_point

    def get_yaw_two_point(self, source_point, target_point):
        # calculate arctan(in rad) of two point
        dx = target_point.x - source_point.x
        dy = target_point.y - source_point.y
        yaw = math.atan(dy/dx) * 180/math.pi
        #print 'original yaw', yaw
        #print 'dx', dx
        #print 'dy', dy
        # rad compensation
        if self.switch == False:
            if( dx > 0 and dy > 0):
                yaw = yaw
            elif( dx < 0):
                yaw = yaw + 180
            elif( dx > 0 and dy < 0):
                yaw = yaw + 360
        else:
            if( dx > 0):
                yaw = yaw
            elif( dx < 0 and dy > 0):
                yaw = yaw + 180
            elif( dx < 0 and dy < 0):
                yaw = yaw - 180
            elif( dx == 0 and dy == 0):
                yaw = 0
            elif( dx == 0 and dy > 0):
                yaw = 90
            elif( dx == 0 and dy < 0):
                yaw = -90 
        return yaw

    def get_dist_two_point(self, source_point, target_point):
        dx = target_point.x - source_point.x
        dy = target_point.y - source_point.y
        dist = math.sqrt(dx * dx + dy * dy)
        return dist 
    def set_pre_vehicle_point(self, point):
        self.pre_vehicle_point.x = point.x
        self.pre_vehicle_point.y = point.y
        self.pre_vehicle_point.z = point.z

    def publish_car_cmd(self, v, omega, duration):
        # publish car command
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.pub_car_cmd.publish(car_cmd_msg)
        rospy.sleep(duration)
        # stop 1s
        #car_cmd_msg.v = 0
        #car_cmd_msg.omega = 0
        #self.pub_car_cmd.publish(car_cmd_msg)
        #rospy.sleep(0)      

    def onShutdown(self):
        # Send stop command
        self.publish_car_cmd(0,0,2)
        rospy.loginfo("[%s] Shutdown" %self.node_name)

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

if __name__ == '__main__':
    rospy.init_node('mocap_waypoint_planning_node',anonymous=False)
    mocap_waypoint_planning_node = MocapWaypointPlanningNode()
    rospy.on_shutdown(mocap_waypoint_planning_node.onShutdown)
    rospy.spin()
