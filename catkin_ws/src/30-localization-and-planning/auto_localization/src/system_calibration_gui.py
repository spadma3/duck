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
import time

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
        self.TDA_height = self.screen_height - 2*self.frame_size - self.TDA_level_title - self.TDA_completion_title - self.block_space
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
        self.WDA_height = self.screen_height - 2*self.frame_size - self.block_space - self.ex_img_height - self.TDA_level_title
        self.WDA_origin_x = self.ex_img_origin_x
        self.WDA_origin_y = self.ex_img_origin_y + self.ex_img_height + self.block_space
        ##

        ## React Area
        self.React_length = self.TDA_length
        self.React_height = self.TDA_completion_title
        self.React_origin_x = self.TDA_origin_x
        self.React_origin_y = self.TDA_origin_y + self.TDA_height + self.block_space

        ## Set screen (pygame surface)
        self.screen_TDA = pg.Surface((self.TDA_length, self.TDA_height))
        self.screen_TDA.fill(WHITE)

        self.screen.blit(self.ex_img, [self.ex_img_origin_x, self.ex_img_origin_y])

        self.screen_WDA = pg.Surface((self.WDA_length, self.WDA_height))
        self.screen_WDA.fill(WHITE)

        self.screen_React = pg.Surface((self.React_length, self.React_height))
        self.screen_React.fill(WHITE)
        ##

        ## Draw all Watchtowers in WDA
        self.WT_block = self.WDA_length / 5
        self.draw_WDA({}, {})
        ##
        ################################################################################

        self.screen.blit(self.screen_TDA, [self.TDA_origin_x, self.TDA_origin_y])
        self.screen.blit(self.screen_WDA, [self.WDA_origin_x, self.WDA_origin_y])
        self.screen.blit(self.screen_React, [self.React_origin_x, self.React_origin_y])
        pg.display.flip()

    def draw_status(self, level, tags_tree, left_watchtowers):

        ## Clear old output
        self.screen_TDA.fill(WHITE)
        self.screen_WDA.fill(WHITE)
        ##

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
        position_x = origin_x
        position_y = 0
        dictPosition = 1
        dictItemCount = len(tree_level['watchtowers'])
        for watchtower in tree_level['watchtowers']:
            height_WT_image = self.draw_watchtower(self.screen_TDA, watchtower[-2:], tree_level['watchtowers'][watchtower], self.WT_block, position_x, position_y)
            ## Determines the x, y of next watchtower image
            position_y += height_WT_image

            if position_y + self.WT_block > self.TDA_height and dictPosition != dictItemCount:
                i+=1
                position_y=0
                position_x = i*self.WT_block
            ##
            dictPosition += 1

        length = 0
        x = position_x + self.WT_block
        y = 0
        j = 0
        for tag in tree_level['tags']:
            tag_text = str(tag)
            if font_tags.size(tag_text)[0] > length:
                length = font_tags.size(tag_text)[0]
            self.screen_TDA.blit(font_tags.render(tag_text, True, BLACK), [x, y])
            y += font_tags.size(tag_text)[1]

        return length + x

    def draw_WDA(self, linked_watchtowers, left_watchtowers):

        i = 0
        indence = 5
        position_x = 0
        position_y = indence
        for watchtower in self.watchtowers:
            if watchtower in list(linked_watchtowers):
                continue

            if watchtower in list(left_watchtowers):
                height_WT_image = self.draw_watchtower(self.screen_WDA, watchtower[-2:], left_watchtowers[watchtower], self.WT_block, position_x, position_y)
            else:
                height_WT_image = self.draw_watchtower(self.screen_WDA, watchtower[-2:], [], self.WT_block, position_x, position_y)

            ## Determines the x, y of next watchtower image
            position_y += height_WT_image

            if position_y + self.WT_block > self.WDA_height:
                i+=1
                position_y=indence
                position_x = i*self.WT_block
            ##

    def draw_React(self):
        self.screen_React.fill(WHITE)



    def draw_watchtower(self, screen_master, watchtower, tags, size, origin_x, origin_y):

        font_tag_size = size / 3
        font_tag = pg.font.SysFont(None, int(font_tag_size))
        font_tag_height = font_tag.size(str(000))[1]

        length = size
        number_of_words = 4
        if font_tag_height * len(tags) > size:
            height = font_tag_height * len(tags)
        else:
            height = size
        screen_WT = pg.Surface((int(length), int(height)))
        screen_WT.fill(GRAY)

        # Write the tags that this watchtower has seen
        i = 0
        for tag in tags:
            screen_WT.blit(font_tag.render(str(tag), True, BLACK), [int(length / 2)+5, font_tag_height*i])
            i+=1
        ##

        # Drawing a ellipse represent the watchtower
        ellipse_length = int(length / 2)
        ellipse_height = height
        pg.draw.ellipse(screen_WT, BLUE, pg.Rect(0, 0, ellipse_length, ellipse_height))

        font_wt = pg.font.SysFont(None, size / 3)
        font_wt_origin_x = 2
        font_wt_origin_y = ellipse_height / 2 - (ellipse_height/5) / 2
        screen_WT.blit(font_wt.render("W" + str(watchtower), True, BLACK), [int(font_wt_origin_x), int(font_wt_origin_y)])
        ##

        screen_master.blit(screen_WT, [origin_x, origin_y])

        return height


class system_calibration_gui(object):

    def __init__(self):

        # set parameters to wait a few moments after finishing befor calibrating
        self.wait_for_message = 15 # At least wait 3 secs for tags collection after all watchtower have publish things.
        self.deadline = time.time() + 100000 # set deadline really high at start, will be set to actual value later
        self.ready = False


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
        self.pub_tfs = rospy.Publisher("~complete_local_poses", RemapPoseArray, queue_size=1)

        self.poses = []

        self.finish_calibration = False

    def callback(self, msg_poses):

        ## After all watchtowers are in the tree, only publish the poses once.
        if self.finish_calibration:
            return

        # Update the status of watchtower, see if it's connected in the link tree
        # Return True if all watchtowers are connected
        self.poses.extend(msg_poses.poses)
        self.tags_tree = []
        if self.ready == False:
            self.ready = self.update_watchtower_status(self.poses)
            if self.ready == True:
                self.deadline = time.time() + self.wait_for_message

        elif time.time() > self.deadline:
            self.pub_tfs.publish(self.poses)
            self.finish_calibration = True
        else:
            sys.stdout.write('\rStart Calibration in %s secs'  % (self.deadline - time.time()))
            sys.stdout.flush()
            #rospy.loginfo("Start Calibration in %d secs", (self.deadline - time.time()))

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
        for watchtower in all_watchtowers.copy(): ## Can't change dictionary size while iterating. So use dict.copy().
            if self.map_origins['id'] in all_watchtowers[watchtower]:
                self.add_watchtower_to_tree(0, watchtower, all_watchtowers[watchtower])
                del all_watchtowers[watchtower]
        if len(self.tags_tree) > 0:
            keep_build = True

        level = 0
        while keep_build:
            level += 1

            all_watchtowers_original = all_watchtowers.copy() # To compare after iteration. Also can't change dictionary size while iterating. So create a clone for iteration.
            for watchtower in all_watchtowers.copy():
                if self.connected_to_tree(level, all_watchtowers[watchtower]):
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
        print "tags_tree", self.tags_tree
        print "rest_watchtowers", all_watchtowers
        self.gui.draw_status(level, self.tags_tree, all_watchtowers)

        watchtower_list = []
        for l in self.tags_tree:
            watchtower_list.extend(list(l['watchtowers'].keys()))

        if all_watchtowers == {} and set(self.map_watchtowers) == set(watchtower_list):
            return True
        else:
            return False

    def add_watchtower_to_tree(self, level, watchtower, tags):

        tree_level = {}
        tree_level['watchtowers'] = {}
        tree_level['tags'] = []
        if level == len(self.tags_tree):
            self.tags_tree.append(tree_level)
        elif level > len(self.tags_tree):
            rospy.loginfo("There are something wrong here!!!")
            pass

        self.tags_tree[level]['watchtowers'][watchtower] = tags
        for tag in tags:
            if not tag in self.tags_tree[level]['tags']:
                self.tags_tree[level]['tags'].append(tag)

    def connected_to_tree(self, level, tags):

        if set(self.tags_tree[level-1]['tags']).intersection(set(tags)):
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
