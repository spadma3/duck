#!/usr/bin/env python

## AIDO localization Sysem Calibration
# Author: Chen-Lung (Eric) Lu , ETHZ NCTU, eric565648.eed03@g2.nctu.edu.tw

## This script records the positions of all reference Apriltags
# and save them into map file
# The scenerio is that the system will calibrate itself from time to time.

import rospkg
import rospy
import sys
import yaml
import numpy as np
import pygame as pg

from duckietown_msgs.msg import RemapPoseArray, RemapPose

class status_gui(object):

    def __init__(self, watchtowers):
        pass


class system_calibration_gui(object):

    def __init__(self):

        self.node_name = 'system_calibration_gui'

        # load the map file
        self.map_filename = rospy.get_param("~map") + ".yaml"
        self.map_data = self.load_map_info(self.map_filename)

        ####### Varaibles and Parameters ##########

        ### Trees
        ## self.tags_tree
        # The tag tree is a list store the whole link of tags
        # Each index of the list equals to the level of the tree
        # Each level of the tree has a self.tree_level dictionary store the data of the level
        self.tags_tree = []
        ## self.tree_level
        # The tree_level variable has index 'watchtowers' and 'tags'
        # The index 'watchtowers' is a list contains multiple self.watchtower dictionary which store data of the watchtowers
        # The index 'tags' is a list which store all tags of this level
        self.tree_level = {}
        self.tree_level['watchtowers'] = {}
        self.tree_level['tags'] = []
        ## self.watchtower
        # The watchtower is a dictionary has index 'name' and 'tags'
        # Index 'name' specify the name of watchtower
        # Index 'tags' has data of the tags that the watchtower saw
        # self.watchtower = {}
        # self.watchtower['name'] = None
        # self.watchtower['tags'] = []

        ### Status drawing class ###
        self.gui = status_gui(self.map_watchtowers)

        ########################################


        # Subscribe all tfs from subfserver node
        self.sub_tfs = rospy.Subscriber("local_poses", RemapPoseArray, self.callback, queue_size=1)

        # Publish result to system calibration node
        self.pub_tfs = rospy.Publisher("complete_local_poses", RemapPoseArray, queue_size=1)

    def callback(self, msg_poses):

        # Update the status of watchtower, see if it's connected in the link tree
        # Return True if all watchtowers are connected
        ready = self.update_watchtower_status(msg_poses.poses)

        if ready:
            self.pub_tfs(msg_poses)

    def update_watchtower_status(self, poses):

        # Manage watchtower datas
        all_watchtowers = {}
        for pose in poses:
            if not pose.host in all_watchtowers:
                all_watchtowers[pose.host] = []
            if not pose.frame_id in all_watchtowers[pose.host]:
                all_watchtowers[pose.host].append(pose.frame_id)

        ############# Start building the tree ####################
        keep_build = False

        # From the origin tag
        for watchtower in all_watchtowers:
            if self.map_origins['id'] in all_watchtowers[watchtower]:
                self.add_watchtower_to_tree(0, watchtower, all_watchtowers[watchtower])
                del all_watchtowers[watchtower]
        if len(self.tags_tree) > 0:
            keep_build = True

        level = 0
        while keep_build:
            level += 1

            all_watchtowers_original = all_watchtowers
            for watchtower in all_watchtowers:
                if self.connected_to_tree(all_watchtowers[watchtower]):
                    self.add_watchtower_to_tree(level, watchtower, all_watchtowers[watchtower])
                    del all_watchtowers[watchtower]

            if all_watchtowers == all_watchtowers_original:
                break
        ##########################################################

        # Draw the result after building the tree
        self.draw_status(level, all_watchtowers)

        watchtower_list = []
        for l in self.tags_tree:
            watchtower_list.extend(list(l['watchtowers'].keys()))

        if all_watchtowers == {} and set(self.map_watchtowers) == set(watchtower_list):
            return True
        else:
            return False

    def add_watchtower_to_tree(self, level, watchtower, tags):

        if level == len(self.tags_tree):
            self.tags_tree.append(self.tree_level)
        elif level > len(self.tags_tree):
            rospy.loginfo("There are something wrong here!!!")
            pass

        self.tags_tree[level]['watchtowers'][watchtower] = tags
        for tag in tags:
            if not tag in self.tags_tree[level]['tags']:
                self.tags_tree[level]['tags'].append(tag)

    def connected_to_tree(self, tags):

        if set(self.tags_tree[level]['tags']).intersection(set(tags)):
            return True
        else:
            return False

    ## Load Map Data
    def load_map_info(self, filename):

        map_data = yaml.load(file(rospkg.RosPack().get_path('auto_localization')+"/config/"+filename,'r')) # Need RosPack get_path to find the file path
        print "Loaded map from file: ", self.map_filename

        self.map_tiles = map_data['tiles']
        print "\nThis is your map: \n", self.map_tiles

        self.map_watchtowers = map_data['watchtowers']
        print "\nThese watchtowers suppose to work: \n", self.map_watchtowers

        self.map_origins = map_data['origin']
        print "The origins: \n", self.map_origins

        return map_data


### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('system_calibration_gui',anonymous=False)
    node = system_calibration()
    rospy.spin()
