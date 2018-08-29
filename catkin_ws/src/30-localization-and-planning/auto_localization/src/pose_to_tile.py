#!/usr/bin/env python

#this script maps pose to tile




import rospkg
import rospy
import sys
import yaml
import numpy as np
from datetime import datetime
import tf
import tf.transformations as tr
import numpy as np
import time
import global_pose_functions as gposf
from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose
from geometry_msgs.msg import PoseStamped, Pose2D


class tilemapping (object):


    def __init__(self):

        self.node_name = 'tilemapping'

        # Statistics of Realism in meters
        tiles_length = 0.585
        tiles_border = 0.0125
        tiles_white_length = 0.044
        duckiebot_size = 0.03

        ### The 2D pose of origin tag reference to the map
        self.origin_tag_pos = Pose2D()
        self.origin_tag_pos.x = 0
        self.origin_tag_pos.y = 0
        self.origin_tag_pos.theta = 0 # in degrees

        # load the map file, notice that the file will be overwritten after calibration
        # self.map_filename = rospy.get_param("~map") + ".yaml"
        # timestr = "{:%Y%m%d-%H%M}".format(datetime.now())
        # self.output_map_filename = rospy.get_param("~output_file") + timestr + ".yaml"
        # self.map_data = self.load_map_info(self.map_filename)

        #open map_description
        map_file = open (map_description.py,r)

        # Subscribe to global poses
        self.sub_globalposes = rospy.Subscriber("global_poses", GlobalPoseArray, self.callback, queue_size=1)
        print "Init complete"
        #self.pub_currenttile = rospy.Publisher("current_tile",GlobalPoseArray,self.callback, queue_size=1)
        #


        #  def callback tilepose
    def callback(self,msg):
        tiles_length = 0.585
        msg_tfs = msg.poses
        for globalpose in msg_tfs:
            print globalpose.pose.x
            print globalpose.pose.y
            print globalpose.pose.theta
        #calculate tile from x and y absolute position
        tile_x = int((globalpose.pose.x)/tiles_length)+1
        tile_y = -int((globalpose.pose.y)/tiles_length)+1
        print tile_x
        print tile_y

        #find the type of the tile from tile numbers in x and y
        string current_tiletype
        if tile_x == && tile_y ==
            current_tiletype = "4-way intersection"
        elif tile_x == && tile_y ==
            current_tiletype = "3-way intersection"
        elif tile_x == && tile_y ==
            current_tiletype = "curve"
        elif tile_x == && tile_y ==
            current_tiletype = "straight"
        pub_currenttile.publish (current_tiletype)










### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('tilemapping',anonymous=False, disable_signals=True)
    node = tilemapping()
    rospy.spin()
