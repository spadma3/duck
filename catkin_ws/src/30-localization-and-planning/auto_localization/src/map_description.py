#!/usr/bin/env python

import rospkg
import rospy
import yaml
import pygame as pg

from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose
from geometry_msgs.msg import PoseStamped, Pose2D

DUCKIE_COLOR = (255, 173, 51)
GRAY = (153, 153, 153)
BLUE = (0, 0, 255)
RED = (255, 0, 0)

class map_description(object):

    def __init__(self):

        ###### Load Map from yaml ######
        self.map_filename = rospy.get_param("~map", "eth_robotarium") + ".yaml"
        self.map_tiles = self.load_map()
        self.map_tiles_img = [] # 2D array store images of tiles
        self.map_shape = [len(self.map_tiles), len(self.map_tiles[0])]

        self.tile_image_file = rospkg.RosPack().get_path('duckietown_description')+"/urdf/meshes/tiles/"
        ################################

        ###### Parameters set up #############

        # The 2D pose of origin tag reference to the map
        self.origin_tag_pos = Pose2D()
        self.origin_tag_pos.x = 0
        self.origin_tag_pos.y = 0
        self.origin_tag_pos.theta = 0 # in degrees

        # Subscriber
        self.sub_bot = rospy.Subscriber("bot_pose", Pose2D, self.update_map_callback, queue_size=1)

        ######################################

        ###### Initial pygame and display #####
        pg.init()
        pg.display.set_caption("Duckietown Robotarium")

        ### Parameters of drawing
        self.screen_height = 900
        self.screen_length = 1200
        self.frame_size = 20
        self.block_space = 10

        ### Set screen (surface)
        self.screen = pg.display.set_mode((self.screen_length, self.screen_height))
        self.screen.fill(DUCKIE_COLOR)

        ### Initial Draw map

        self.block_size = 64 # The size per tile in pixel

        # Initial Map tile image
        row_i = 0
        col_i = 0
        for row_i in range(self.map_shape[0]):
            self.map_tiles_img.append([])
            for col_i in range(self.map_shape[1]):
                # Load tiles image to an array
                self.map_tiles_img[row_i].append(self.load_tile_image(self.map_tiles[row_i][col_i]))

        self.draw_map()

        ### Draw

        ## Update screen after all setup
        pg.display.flip()

    def update_map_callback(self, bot_poses):
        pass

    def draw_map(self):

        # Draw Map
        for row_i in range(self.map_shape[0]):
            for col_i in range(self.map_shape[1]):
                # Draw tile image on screen
                self.screen.blit(self.map_tiles_img[row_i][col_i], [col_i*self.block_size+self.frame_size, row_i*self.block_size+self.frame_size])

        # Draw Coordination
        self.coor_l = 80
        self.coor_w = 10
        #pg.draw.rect(self.screen, BLUE, [self.frame_size, self.frame_size-self.coor_w/2, self.coor_l, self.coor_w])
        pg.draw.aaline(self.screen, BLUE, [self.frame_size, self.frame_size-self.coor_w/2], [self.frame_size+self.coor_l, self.frame_size-self.coor_w/2], width=50)
        pg.draw.rect(self.screen, RED, [self.frame_size-self.coor_w/2, self.frame_size, self.frame_size+self.coor_w/2, self.frame_size+self.coor_l])

        # General Data Area
        self.GDA_height = 340
        self.GDA_length = 510
        self.GDA_origin_x = self.frame_size + self.block_size * self.map_shape[1] + self.block_space
        self.GDA_origin_y = self.frame_size

        pg.draw.rect(self.screen, GRAY, [self.GDA_origin_x, self.GDA_origin_y, self.GDA_length, self.GDA_height])

        # Robot Data Area
        self.RDA_height = 510
        self.RDA_length = 510
        self.RDA_origin_x = self.GDA_origin_x
        self.RDA_origin_y = self.GDA_origin_y + self.GDA_height + self.block_space

        pg.draw.rect(self.screen, GRAY, [self.RDA_origin_x, self.RDA_origin_y, self.RDA_length, self.RDA_height])

    def draw_general_data(self, arg):
        pass

    def draw_robot_data(self, arg):
        pass

    def load_tile_image(self, image):

        this_tile = None
        if image == "grass":
            this_tile = pg.image.load(self.tile_image_file + "empty.png").convert()
        elif image == "4way":
            this_tile = pg.image.load(self.tile_image_file + "4way.png").convert()
        elif image[:-2] == "3way_left":
            this_tile = pg.image.load(self.tile_image_file + "3way.png").convert()
        elif image[:-2] == "straight":
            this_tile = pg.image.load(self.tile_image_file + "straight.png").convert()
        elif image[:-2] == "curve_left":
            this_tile = pg.image.load(self.tile_image_file + "turn.png").convert()

        this_tile = pg.transform.scale(this_tile, (self.block_size, self.block_size))
        direction = image[-1]
        if direction == "E":
            this_tile = pg.transform.rotate(this_tile, 0)
        elif direction == "N":
            this_tile = pg.transform.rotate(this_tile, 90)
        elif direction == "W":
            this_tile = pg.transform.rotate(this_tile, 180)
        elif direction == "S":
            this_tile = pg.transform.rotate(this_tile, 270)

        return this_tile

    def load_map(self):

        # Load map data with yaml
        map_data = yaml.load(file(rospkg.RosPack().get_path('auto_localization')+"/config/"+self.map_filename,'r'))

        # Load tiles, it's a 2D array containing map in form of different kinds of tiles
        self.map_tiles = map_data['tiles']

        return self.map_tiles


if __name__ == '__main__':
    rospy.init_node('map_description_node', anonymous=False)
    node = map_description()
    rospy.spin()
