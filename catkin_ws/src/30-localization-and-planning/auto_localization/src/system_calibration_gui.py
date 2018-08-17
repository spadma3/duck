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

#### Static Parameters for Drawing ######
DUCKIE_COLOR = (255, 173, 51)
GRAY = (153, 153, 153)
BLUE = (153, 204, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

DUCKIEBOT_COLOR = (0, 0, 255)
DELTA_XY_COLOR = (0, 0, 255) # will be transparant
DELTA_THETA_COLOR = (0, 0, 255)

pg.font.init()
font_data = pg.font.SysFont(None, 25)
font_title = pg.font.SysFont(None, 40)
font_tags = pg.font.SysFont(None, 20)

# FPS of screen flip
fps = 30
clock = pg.time.Clock()

##########################################

class status_gui(object):

    def __init__(self, watchtowers):

        ####### Initial pygame and display ##########################################
        pg.init()
        pg.display.set_caption("Duckietown System Calibration")

        ### Parameters of drawing
        self.screen_height = 900
        self.screen_length = 1200
        self.frame_size = 20
        self.block_space = 10

        self.screen = pg.display.set_mode((self.screen_length, self.screen_height))
        self.screen.fill(DUCKIE_COLOR)

        self.watchtowers = watchtowers

        ## TDA = Tree Drawing Area
        # TDA Titles
        self.TDA_level_title = 30
        self.TDA_completion_title = 60

        # TDA Screen
        self.TDA_height = self.screen_height - 2*self.frame_size - self.TDA_level_title - self.TDA_completion_title
        self.TDA_length = 900
        self.TDA_origin_x = self.frame_size
        self.TDA_origin_y = self.frame_size + self.TDA_level_title
        ##

        ## Example Image
        self.ex_img = pg.image.load(rospkg.RosPack().get_path('auto_localization')+"/map_image/duckiebot.png").convert()
        self.ex_img_length = self.screen_length - self.TDA_length - 2*self.frame_size - self.block_space
        self.ex_img_height = self.ex_img_length * self.ex_img.get_rect().size[1] / self.ex_img.get_rect().size[1] # To resize the example image in ratio
        self.ex_img = pg.transform.scale(self.ex_img, (self.ex_img_length, self.ex_img_height))
        self.ex_img_origin_x = self.TDA_origin_x + self.TDA_length + self.block_space
        self.ex_img_origin_y = self.TDA_origin_y
        ##

        ## WDA = (not in tree) Watchtowers Drawing Area
        self.WDA_length = self.ex_img_length
        self.WDA_height = self.screen_height - 2*self.frame_size - self.block_space - self.ex_img_height - self.TDA_level_title - self.TDA_completion_title
        self.WDA_origin_x = self.ex_img_origin_x
        self.WDA_origin_y = self.ex_img_origin_y + self.ex_img_height + self.block_space
        ##

        ## Set screen (pygame surface)
        self.screen_TDA = pg.Surface((self.TDA_length, self.TDA_height))
        self.screen_TDA.fill(WHITE)

        self.screen.blit(self.ex_img, [self.ex_img_origin_x, self.ex_img_origin_y])

        self.screen_WDA = pg.Surface((self.WDA_length, self.WDA_height))
        self.screen_WDA.fill(WHITE)
        ##

        ## Draw all Watchtowers in WDA
        self.WT_block = self.WDA_length / 5
        self.draw_WDA({}, {})
        ##
        ################################################################################

        self.screen.blit(self.screen_TDA, [self.TDA_origin_x, self.TDA_origin_y])
        self.screen.blit(self.screen_WDA, [self.WDA_origin_x, self.WDA_origin_y])
        pg.display.flip()

    def draw_status(self, level, tags_tree, left_watchtowers):

        length = 0
        linked_watchtowers = {}
        for tree_level in tags_tree:
            length = self.draw_TDA(tree_level, length) # the function return how much length it used for drawing next level
            linked_watchtowers.update(tree_level['watchtowers'])

        self.draw_WDA(linked_watchtowers, left_watchtowers)

        self.screen.blit(self.screen_TDA, [self.TDA_origin_x, self.TDA_origin_y])
        self.screen.blit(self.screen_WDA, [self.WDA_origin_x, self.WDA_origin_y])

        pg.display.flip()

    def draw_TDA(self, tree_level, origin_x):

        i = 0
        j = 0
        for watchtower in tree_level['watchtowers']:
            x = origin_x + i*self.WT_block
            y = j * self.WT_block
            self.draw_watchtower(self.screen_TDA, watchtower[-2:], tree_level['watchtowers'][watchtower], self.WT_block, x, y)
            j+=1
            if j*WT_block+WT_block > self.TDA_height:
                i+=1
                j=0

        length = 0
        x = origin_x + (i+1)*self.WT_block
        y = 0
        j = 0
        for tag in tree_level['tags']:
            tag_text = str(tag)
            if font_tags.size(tag_text)[0] > length:
                length = font_tags.size(tag_text)[0]
            self.screen_TDA(font_tags.render(tag_text, True, BLACK), [x, y])
            y += font_tags.size(tag_text)[1]

        return length + x


    def draw_WDA(self, linked_watchtowers, left_watchtowers):

        i = 0
        j = 0
        indence = 5
        for watchtower in self.watchtowers:
            if watchtower in list(linked_watchtowers):
                continue

            x = i*self.WT_block
            y = j*self.WT_block + indence ## y is the hard coded with a side blanck
            if watchtower in list(left_watchtowers):
                self.draw_watchtower(self.screen_WDA, watchtower[-2:], left_watchtowers[watchtower], self.WT_block, x, y)
                j+=1
            else:
                self.draw_watchtower(self.screen_WDA, watchtower[-2:], [], self.WT_block, x, y)

            j+=1
            if j*self.WT_block + indence + self.WT_block > self.WDA_height:
                i+=1
                j=0

    def draw_watchtower(self, screen_master, watchtower, tags, size, origin_x, origin_y):

        screen_WT = pg.Surface((int(size), int(size)))
        screen_WT.fill(GRAY)

        # Drawing a ellipse represent the watchtower
        ellipse_length = int(size / 2)
        ellipse_height = size
        pg.draw.ellipse(screen_WT, BLUE, pg.Rect(0, 0, ellipse_length, ellipse_height))

        font_wt = pg.font.SysFont(None, ellipse_height / 3)
        font_wt_origin_x = 2
        font_wt_origin_y = ellipse_height / 2 - (ellipse_height/5) / 2
        screen_WT.blit(font_wt.render("W" + str(watchtower), True, BLACK), [int(font_wt_origin_x), int(font_wt_origin_y)])
        ##

        # Write the tags that this watchtower has seen
        if len(tags) <= 5:
            font_tag_size = ellipse_height / 5
        else:
            font_tag_size =  len(tags)

        pg.font.SysFont(None, font_tag_size)

        i = 0
        for tag in tags:
            screen_WT.blit(font_wt.render(str(tag), True, BLACK), [ellipse_length + 5, font_tag_size*i])
            i+=1
        ##

        screen_master.blit(screen_WT, [origin_x, origin_y])


class system_calibration_gui(object):

    def __init__(self):

        self.node_name = 'system_calibration_gui'

        # load the map file
        self.map_filename = rospy.get_param("~map", "eth_robotarium") + ".yaml"
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
        # self.watchtower is a list has 'tags'
        # self.watchtower = []
# Index 'name' specify the name of watchtower
        # Index 'tags' has data of the tags that the watchtower saw
        ### Status drawing class ###
        self.gui = status_gui(self.map_watchtowers)

        ########################################


        # Subscribe all tfs from subfserver node
        self.sub_tfs = rospy.Subscriber("local_poses", RemapPoseArray, self.callback, queue_size=1)

        # Publish result to system calibration node
        self.pub_tfs = rospy.Publisher("complete_local_poses", RemapPoseArray, queue_size=1)

        self.poses = []

    def callback(self, msg_poses):

        # Update the status of watchtower, see if it's connected in the link tree
        # Return True if all watchtowers are connected
        self.poses.extend(msg_poses.poses)
        ready = self.update_watchtower_status(self.poses)

        if ready:
            self.pub_tfs(msg_poses)

    def update_watchtower_status(self, poses):

        if poses == []:
            return False

        # Manage watchtower datas
        # all_watchtowers = a dictionary with watchtower name as index.
        # For the specific index, it has a list which includes tags that the watchtower has seen.
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
        # level = the total level that the tree has now
        # self.tags_tree = A list that specify the watchtowers and tags of each level (the indexes are the level of the tree)
        # all_watchtowers = Originally all_watchtowers has all watchtowers datas. However during the process of building the tree,
        # the data in all_watchtowers will be deleted if the watchtower was in the tree. The left watchtowers in all_watchtowers are watchtowers
        # that did not link to the tree
        self.gui.draw_status(level, self.tags_tree, all_watchtowers)

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

        self.map_origins = map_data['origin'][0]
        print "The origins: \n", self.map_origins

        return map_data


### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('system_calibration_gui',anonymous=False)
    node = system_calibration_gui()
    rospy.spin()
