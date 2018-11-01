#!/usr/bin/env python

## AIDO localization server side postprocessing
# Author: Josefine Quack, ETHZ, jquack@ethz.ch

## This script converts relative positions of the localization tool
# into a abslute positions


## coordinates: Origin in top left corner, theta counterclockwise from x-axis
#  y
#  ^
#  |
#  |
#  O -----> x

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
import math
import global_pose_functions as gposf
import copy

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
        self.optimized_pose = GlobalPose()
        self.optimized_pose.bot_id = bot_id

        self.time_stamp_width = 0.02 # unit: s. The width of time that seen as the same time stamp. Ex: current_time_stamp == current_time_stamp-time_stamp_width

    def add_pose(self, pose):
        # pose is a GlobalPose

        pose_time = pose.header.stamp.to_sec() # Unit: s

        if pose_time < self.current_time_stamp - self.time_stamp_width:
            # We don't use the time stamp which is too far away from current time
            return
        elif pose_time < self.current_time_stamp:
            pass
        else: # if pose_time > current_time_stamp:
            self.current_time_stamp = pose_time

        # Sort the pose according to its time stamp
        if not str(pose_time) in self.poses:
            self.poses[str(pose_time)] = []
        self.poses[str(pose_time)].append(pose)

        # Delete time stamps which are out of date
        for time_stamp in self.poses.copy(): ## Can't change dictionary size while iterating. So use dict.copy().
            if self.current_time_stamp - self.time_stamp_width > float(time_stamp):
                del self.poses[time_stamp]

    def get_optimized_pose(self):

        poses_x = []
        poses_y = []
        poses_theta = []

        self.camera_id = []
        self.reference_tag_id = []

        for time_stamp in self.poses:
            for a_pose in self.poses[time_stamp]:
                poses_x.append(a_pose.pose.x)
                poses_y.append(a_pose.pose.y)
                poses_theta.append(a_pose.pose.theta)
                # camera id's which saw this Duckiebot
                if not a_pose.cam_id[0] in self.camera_id:
                    self.camera_id.append(a_pose.cam_id[0])
                # Reference tags which used by this detection
                if not a_pose.reference_tag_id[0] in self.reference_tag_id:
                    self.reference_tag_id.append(a_pose.reference_tag_id[0])

        self.optimized_pose.pose.x = np.mean(poses_x)
        self.optimized_pose.delta_x = np.sqrt(np.mean(poses_x - self.optimized_pose.pose.x)**2) # Take RMSE of pose_x for delta_x
        self.optimized_pose.pose.y = np.mean(poses_y)
        self.optimized_pose.delta_y = np.sqrt(np.mean(poses_y - self.optimized_pose.pose.y)**2) # Take RMSE of pose_y for delta_y
        # Need some special way to calculate mean for angles
        # The range of input angle is +- pi
        self.optimized_pose.pose.theta = self.average_angle(poses_theta)
        self.optimized_pose.delta_theta = self.rms_angle(poses_theta, self.optimized_pose.pose.theta)

        self.optimized_pose.cam_id = self.camera_id
        self.optimized_pose.reference_tag_id = self.reference_tag_id

        self.optimized_pose.header.stamp = rospy.Time(int(self.current_time_stamp), self.current_time_stamp - int(self.current_time_stamp))

        return copy.deepcopy(self.optimized_pose) # python always di reference. Thus we don't wanna self.optimized_pose to be modified.

    def average_angle(self, thetas):

        #Ref
        # https://en.wikipedia.org/wiki/Mean_of_circular_quantities
        # https://greek0.net/blog/2016/06/14/working_with_angles/

        x = 0
        y = 0
        for theta in thetas:
            x += math.cos(theta)
            y += math.sin(theta)

        if x == 0:
            return math.copysign(math.pi/2, y)

        return math.atan2(y, x)

    def rms_angle(self, thetas, mean):

        # It's the RMSE of angles
        # sqrt(sum d(angle_i, mean)^2)
        # d(angle1, angle2) = acos(cos(angle1-angle2))

        d_sq = []

        for theta in thetas:
            d = math.acos(math.cos(theta - mean))
            d_sq.append(d**2)

        return np.sqrt(np.mean(d_sq))

class pose_optimization(object):
    """"""
    def __init__(self):
        self.node_name = 'pose_optimization'

        # Open Output .csv file
        self.output_file_name = rospy.get_param("~output_file", "testing") + "_optimize"
        self.output_file = self.init_output_file(self.output_file_name)

        #Add Subscriber
        self.sub_pos = rospy.Subscriber("bot_global_poses", GlobalPoseArray, self.poses_callback, queue_size=1)

        #Add Publisher
        self.pub_opt_pos = rospy.Publisher("~bot_global_poses_optimized", GlobalPoseArray, queue_size=1)

        ### Variable
        # Store Duckiebots
        self.bots = dict()

    def poses_callback(self, poses):

        output_poses = GlobalPoseArray()

        for bot_pose in poses.poses:
            if not str(bot_pose.bot_id) in self.bots:
                self.bots[str(bot_pose.bot_id)] = BotOptimizedPose(bot_pose.bot_id)
            self.bots[str(bot_pose.bot_id)].add_pose(bot_pose)

        for bot in self.bots:
            new_output = self.bots[bot].get_optimized_pose()
            output_poses.poses.append(new_output)

            new_data = [new_output.header.stamp, new_output.bot_id,  new_output.pose.x, new_output.pose.y, new_output.pose.theta, new_output.delta_x, new_output.delta_y, new_output.delta_theta, new_output.cam_id, new_output.reference_tag_id]
            self.write_data_to_output_file(new_data)

        self.pub_opt_pos.publish(output_poses)

### ------------------ INITIALIZATION FUNCTIONS -------------------------#####

    # creates and initilizes the output file
    # INPUT:    none
    # OUTPUT:   output_file object
    def init_output_file(self, filename):
        time = "{:%Y%m%d-%H%M%S}".format(datetime.now())
        filename_dates = filename +time + ".csv"
        output_file_name = rospkg.RosPack().get_path('auto_localization') + "/config/csv/" + filename_dates
        print output_file_name
        output_file = open(output_file_name, 'w+')
        output_file.write('time, bot_ID, x, y, theta, camera_id, reference_tag_id\n')
        return output_file

### --------------------- USER OUTPUT FUNCTIONS --------------------#####
    # writes new data to the output file
    def write_data_to_output_file(self,new_data):
        for idx in xrange( 0, (len(new_data)-1) ):
            self.output_file.write(str(new_data[idx]) + ', ')
        self.output_file.write(str(new_data[len(new_data)-1]) + '\n')



### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('pose_optimization',anonymous=False)
    node = pose_optimization()
    rospy.spin()
