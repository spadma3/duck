#!/usr/bin/env python

## AIDO localization server side postprocessing
# Author: Josefine Quack, ETHZ, jquack@ethz.ch

## This script converts relative positions of the localization tool
# into a abslute positions


## coordinates: Origin in top left corner, theta counterclockwise from x-axis
#  O -----> x
#  |
#  |
#  v  y

## Input:
#   - Global poses compute by absolute_from_relative_position node

## Output:
#    - Optimize pose of Duckiebot
#
#    Statistics file
#    | time | Bot ID | x | y | theta | camera observed | reference Apriltags


import rospkg
import rospy

import yaml
import numpy as np
from datetime import datetime
import tf
import tf.transformations as tr
import global_pose_functions as gposf

from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose

# A class for optimize and managing pose of a Duckiebot
# One could in the future move this class to an independent file so that it could be used in other node.
class BotOptimizedPose(object):
    """"""
    def __init__(self, bot_id):

        ### Varaibles
        self.bot_id = bot_id # The id of the Duckiebot
        self.poses = dict() # Sort the poses according to their time stamp. Each index is a list of GlobalPose
        self.camera_id = [] # The camera that sees the Duckiebot
        self.reference_tag_id = [] # The reference tags that used to localizae the Duckiebot
        self.current_time_stamp = 0 # Current time stamp
        # The variable for output
        self.optmizd_pose = GlobalPose()

        self.time_stamp_width = 20000000 # unit: ns. The width of time that seen as the same time stamp. Ex: current_time_stamp == current_time_stamp-time_stamp_width

    def add_pose(self, pose):
        # pose is a GlobalPose

        pose_time = int(bot.header.stamp.secs) * 1e9 + int(bot.header.stamp.nsecs) # Unit: ns

        if pose_time < current_time_stamp - self.time_stamp_width:
            # We don't use the time stamp which is too far away from current time
            return
        elif pose_time < current_time_stamp:
            pass
        else: # if pose_time > current_time_stamp:
            self.current_time_stamp = pose_time

        # Sort the pose according to its time stamp
        if not str(pose_time) in self.poses:
            self.poses[str(pose_time)] = []
        self.poses[str(pose_time)].append(pose)

        # Delete time stamps which are out of date
        for time_stamp in self.poses:
            if self.current_time_stamp - self.time_stamp_width > int(time_stamp):
                del self.poses[time_stamp]

    def get_optimized_pose(self):

        poses_x = []
        poses_y = []
        poses_theta = []

        for time_stamp in self.poses:
            for a_pose in self.poses[time_stamp]:
                poses_x.append(a_pose.pose.x)
                poses_y.append(a_pose.pose.y)
                poses_theta.append(a_pose.pose.theta)


class pose_optimization(object):
    """"""
    def __init__(self):
        self.node_name = 'pose_optimization'

        # Open Output .csv file
        self.output_file_name = rospy.get_param("~output_file") + "_optimize"
        self.output_file = self.init_output_file(self.output_file_name)

        #Add Subscriber
        self.sub_pos = rospy.Subscriber("bot_global_poses", GlobalPoseArray, self.poses_callback, queue_size=1)

        #Add Publisher
        self.pub_opt_pos = rospy.Publisher("~bot_global_poses_optimized", GlobalPoseArray, queue_size=1)

    def poses_callback(self):
        pass

### ------------------ INITIALIZATION FUNCTIONS -------------------------#####

    # creates and initilizes the output file
    # INPUT:    none
    # OUTPUT:   output_file object
    def init_output_file(self, filename):
        time = "{:%Y%m%d-%H%M%S}".format(datetime.now())
        filename_dates = filename +time + ".csv"
        output_file_name = rospkg.RosPack().get_path('auto_localization') + "/config/" + filename_dates
        print output_file_name
        output_file = open(output_file_name, 'w+')
        output_file.write('time, bot_ID, x, y, theta, camera_id, reference_tag_id\n')
        return output_file



### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('pose_optimization',anonymous=False)
    node = pose_optimization()
    rospy.spin()
