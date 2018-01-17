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
        self.X = [0.0, 0.0, 0.0, 0.0]
        self.Y = [0.0, 0.0, 0.0, 0.0]

        # vehicle point pair
        self.vehicle_yaw_pre = 0
        self.vehicle_front_point = Point()
        self.vehicle_back_point = Point()
        # the previous vehicle point of last moment
        #self.pre_vehicle_point = Point()

        self.kd = 0.01
        self.kp = 0.03

        self.switch = True

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        # Subscription
        self.sub_vehicle_pose_pair = rospy.Subscriber("~vehicle_pose_pair", PoseArray, self.cbPoseArray, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

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

        # set target waypoint position
        target_point = self.set_target_point(self.waypoint_index)

        # calculate yaw angle from vehicle to target waypoint
        target_yaw = self.get_yaw_two_point(self.vehicle_back_point, target_point)

        # calculate yaw angle from vehicle to previous vehicle
        vehicle_yaw = self.get_yaw_two_point(self.vehicle_back_point, self.vehicle_front_point)
        print "fron point", self.vehicle_front_point.x
        print "back point", self.vehicle_back_point.x
        print "target point", target_point.x

        dist = self.get_dist_two_point(self.vehicle_front_point, target_point)

        if(target_yaw >= 180):
            target_taw = 360 - target_yaw
        if(vehicle_yaw >= 180):
            vehicle_taw = 360 - vehicle_yaw
        #self.set_pre_vehicle_point(point_msg)
        print "yaw from vehicle to waypoint: ", target_yaw
        print "yaw from previous vehicle to vehicle: ", vehicle_yaw
        print "distance between vehicle and waypoint", dist

        ess = vehicle_yaw - target_yaw
        diff = vehicle_yaw - self.vehicle_yaw_pre
        self.vehicle_yaw_pre = vehicle_yaw
        u = self.kp * ess + self.kd * diff
        print 'omega pd: ', -u
        if( u < -4):
            u = -4
        if( u > 4):
            u = 4
        print 'mega pd com: ', -u
        if(self.switch):        
            self.publish_car_cmd(0.2, -u , 0.2)
        else:
            self.publish_car_cmd(0, 0, 0.2)
        if(dist <= 0.08):
            self.waypoint_index = self.waypoint_index + 1

    def cbSwitch(self, msg):
        self.switch = msg.data
        if(msg.data):
            print "go on"
        else:
            print "stop"

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
