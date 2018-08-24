#!/usr/bin/env python


######### Coordination #############

# The Coordination here is
#
# ----------------> x
# |
# |
# |
# |
# |
# |
# |
# |
# v
# z
#
# Every Duckiebot pose which sent here should be preprossed before it was sent here.

#######################################

import rospkg
import rospy
import yaml
import pygame as pg
from PIL import Image, ImageDraw
import os, sys
import math

from duckietown_msgs.msg import GlobalPoseArray, GlobalPose
from geometry_msgs.msg import PoseStamped, Pose2D

DUCKIE_COLOR = (255, 173, 51)
GRAY = (153, 153, 153)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

DUCKIEBOT_COLOR = (0, 0, 255)
DELTA_XY_COLOR = (0, 0, 255) # will be transparant
DELTA_THETA_COLOR = (0, 0, 255)

pg.font.init()
font_data = pg.font.SysFont(None, 25)
font_title = pg.font.SysFont(None, 40)

# Statistics of Realism in meters
tiles_length = 0.585
tiles_border = 0.0125
tiles_white_length = 0.044
duckiebot_size = 0.03

# FPS of screen flip
fps = 30
clock = pg.time.Clock()

class map_description(object):

    def __init__(self):

        ###### Load Map from yaml ######
        self.map_filename = rospy.get_param("~map", "eth_robotarium") + ".yaml"
        self.map_tiles = self.load_map()
        self.map_tiles_img = [] # 2D array store images of tiles
        self.map_shape = [len(self.map_tiles), len(self.map_tiles[0])]

        self.tile_image_file = rospkg.RosPack().get_path('duckietown_description')+"/urdf/meshes/tiles/"
        ################################

        ###### Parameters set up ########################################################

        ### The 2D pose of origin tag reference to the map
        self.origin_tag_pos = Pose2D()
        self.origin_tag_pos.x = 0
        self.origin_tag_pos.y = 0
        self.origin_tag_pos.theta = 0 # in degrees

        ### Subscriber
        self.sub_bot = rospy.Subscriber("bot_poses", GlobalPoseArray, self.update_data_callback, queue_size=1)

        ### Varaible for storing Duckiebot data
        # Data of Duckiebots
        self.all_duckiebots = {}

        # Variable for General Data Showing
        self.GDA_data = {} # GDA == General Data Area
        GDA_data_keys = ('bot_num', 'tow_num', 'time')
        self.GDA_data = dict.fromkeys(GDA_data_keys, None)

        # Variable for Specific Robot Data Showing
        self.RDA_data = {} # RDA == Robot Data Area
        # Initail with these keys and value = None
        RDA_data_keys = ('bot_id', 'bot_t', 'x', 'delta_x', 'y', 'delta_y', 'theta', 'delta_theta', 'watchtower', 'ref_tag')
        self.RDA_data = dict.fromkeys(RDA_data_keys, None)

        #############################################################################

        ####### Initial pygame and display ##########################################
        pg.init()
        pg.display.set_caption("Duckietown Robotarium")

        ### Parameters of drawing
        self.screen_height = 900
        self.screen_length = 1200
        self.frame_size = 20
        self.block_space = 10

        self.block_size = 64 # The size per tile in pixel
        self.m2p = self.block_size / tiles_length # The parameter that transfer unit from meters to pixels

        self.GDA_height = 340
        self.GDA_length = 510
        self.GDA_origin_x = self.frame_size + self.block_size * self.map_shape[1] + self.block_space
        self.GDA_origin_y = self.frame_size

        self.RDA_height = 510
        self.RDA_length = 510
        self.RDA_origin_x = self.GDA_origin_x
        self.RDA_origin_y = self.GDA_origin_y + self.GDA_height + self.block_space

        ### Set screen (surface)

        self.screen = pg.display.set_mode((self.screen_length, self.screen_height))
        self.screen.fill(DUCKIE_COLOR)

        self.screen_Map = pg.Surface((self.block_size*self.map_shape[1], self.block_size*self.map_shape[0]))
        self.screen_Map.fill(DUCKIE_COLOR)

        self.screen_GDA = pg.Surface((self.GDA_length, self.GDA_height))
        self.screen_GDA.fill(GRAY)

        self.screen_RDA = pg.Surface((self.RDA_length, self.RDA_height))
        self.screen_RDA.fill(GRAY)

        ### Initial Map tile image
        row_i = 0
        col_i = 0
        for row_i in range(self.map_shape[0]):
            self.map_tiles_img.append([])
            for col_i in range(self.map_shape[1]):
                # Load tiles image to an array
                self.map_tiles_img[row_i].append(self.load_tile_image(self.map_tiles[row_i][col_i]))

        ### Initial Robot Image
        self.bot_img = pg.image.load(rospkg.RosPack().get_path('auto_localization')+"/map_image/duckiebot.png").convert()
        size_in_map = int(duckiebot_size*self.m2p)
        if size_in_map <= 0:
            rospy.loginfo("Size per block is too small!!!")
            self.quit_program()
        self.bot_img = pg.transform.scale(self.bot_img, (size_in_map, size_in_map))

        ### Draw map loop
        self.update_map_loop()

        ############################################################################################

    def update_data_callback(self, bot_poses):

        for bot in bot_poses.poses:
            key_index = str(bot.bot_id)
            if not str(bot.bot_id) in self.all_duckiebots:
                self.all_duckiebots[key_index] = {}
            self.all_duckiebots[key_index]['bot_t'] = bot.header.stamp.secs + float(bot.header.stamp.nsecs)/1e9
            self.all_duckiebots[key_index]['x'] = bot.pose.x
            self.all_duckiebots[key_index]['delta_x'] = bot.delta_x
            self.all_duckiebots[key_index]['y'] = -1*bot.pose.y
            self.all_duckiebots[key_index]['delta_y'] = bot.delta_y
            self.all_duckiebots[key_index]['theta'] = bot.pose.theta
            self.all_duckiebots[key_index]['delta_theta'] = bot.delta_theta
            self.all_duckiebots[key_index]['watchtower'] = bot.cam_id
            self.all_duckiebots[key_index]['ref_tag'] = bot.reference_tag_id

    def update_map_loop(self):

        while True:

            ######## Event Detections ########
            ### Key Detections ###
            keys = pg.key.get_pressed()
            if keys[pg.K_q]:
                self.quit_program()

            ### Pygame Event Detections (inluding mouse detection)###
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.quit_program()
                elif event.type == pg.MOUSEBUTTONDOWN:
                    self.set_RDA(pg.mouse.get_pos())
            ##################################

            ######## Update Map ########
            self.draw_map()
            ############################

            ######## Update Robot ########
            self.draw_duckiebot()
            #############################

            ######### Set fps of screen ########
            clock.tick(fps)
            ### Renew screen ###
            pg.display.flip()
            ####################################

    def set_RDA(self, mouse_pos):

        mouse_x = mouse_pos[0] - self.frame_size # Compensate the offset cause by the border
        mouse_y = mouse_pos[1] - self.frame_size

        for id_key in self.all_duckiebots:
            pixel_x = self.all_duckiebots[id_key]['x'] * self.m2p
            pixel_y = self.all_duckiebots[id_key]['y'] * self.m2p
            range_accept = 25
            if (pixel_x-range_accept <= mouse_x <= pixel_x+range_accept) and (pixel_y-range_accept <= mouse_y <= pixel_y+range_accept):
                self.RDA_data['bot_id'] = id_key

    def draw_map(self):

        ######## Draw Map ########
        for row_i in range(self.map_shape[0]):
            for col_i in range(self.map_shape[1]):
                # Draw tile image on screen
                self.screen_Map.blit(self.map_tiles_img[row_i][col_i], [col_i*self.block_size, row_i*self.block_size])

        # Draw Coordination
        self.coor_l = 80
        self.coor_w = 10
        self.coor_angle = 30
        pg.draw.lines(self.screen, BLUE, False, [[self.frame_size, self.frame_size], [self.frame_size+self.coor_l, self.frame_size], [self.frame_size+self.coor_l-17, self.frame_size-10]], 5) # x-coordinate
        pg.draw.lines(self.screen, RED, False, [[self.frame_size, self.frame_size], [self.frame_size, self.frame_size+self.coor_l], [self.frame_size-10, self.frame_size+self.coor_l-17]], 5) # y-coordinate
        pg.draw.circle(self.screen, BLACK, [self.frame_size, self.frame_size], 8)
        ###################

        ######## Show General Data Area ########
        self.GDA_data['bot_num'] = len(self.all_duckiebots)
        self.GDA_data['tow_num'] = len(self.map_watchtowers)
        now_time = rospy.get_rostime()
        self.GDA_data['time'] = now_time.secs + float(now_time.nsecs)/1e9

        self.draw_general_data()
        ###################

        ######## Show Robot Data Area ########
        if self.RDA_data['bot_id'] == None:
            self.RDA_data = dict.fromkeys(self.RDA_data.iterkeys(), None)
        else:
            bot_id_choose = self.RDA_data['bot_id']
            # Showing difference of time between bot time and current time
            self.RDA_data['bot_t'] = round(self.GDA_data['time'] - self.all_duckiebots[bot_id_choose]['bot_t'], 3)
            self.RDA_data['x'] = round(self.all_duckiebots[bot_id_choose]['x'], 3)
            self.RDA_data['delta_x'] = round(self.all_duckiebots[bot_id_choose]['delta_x'], 3)
            self.RDA_data['y'] = round(self.all_duckiebots[bot_id_choose]['y'], 3)
            self.RDA_data['delta_y'] = round(self.all_duckiebots[bot_id_choose]['delta_y'], 3)
            self.RDA_data['theta'] = round(math.degrees(self.all_duckiebots[bot_id_choose]['theta']), 1)
            self.RDA_data['delta_theta'] = round(math.degrees(self.all_duckiebots[bot_id_choose]['delta_theta']), 1)
            self.RDA_data['watchtower'] = self.all_duckiebots[bot_id_choose]['watchtower']
            self.RDA_data['ref_tag'] = self.all_duckiebots[bot_id_choose]['ref_tag']

        self.draw_robot_data()
        ####################

    def draw_general_data(self):

        # data should be a python dictionary
        self.screen_GDA.fill(GRAY)

        self.GDA_row = 5
        self.GDA_col = 1
        self.GDA_indence = 10
        self.GDA_block_length = (self.GDA_length - 2*self.GDA_indence) / self.GDA_col
        self.GDA_block_height = (self.GDA_height - 2*self.GDA_indence) / self.GDA_row

        text = []
        text.append(font_title.render("General Data", True, BLACK))
        text.append(font_data.render("# of Duckiebots: " + str(self.GDA_data['bot_num']), True, BLACK))
        text.append(font_data.render("# of Watchtowers: " + str(self.GDA_data['tow_num']), True, BLACK))
        text.append(font_data.render("Time Stamp Now: " + str(self.GDA_data['time']), True, BLACK))
        text.append(font_data.render("Metrix System: " + "Meter, Degrees", True, BLACK))

        row_i = 0
        col_i = 0
        for row_i in range(self.GDA_row):
            for col_i in range(self.GDA_col):
                index = row_i*self.GDA_col + col_i
                text_x = col_i*self.GDA_block_length + self.GDA_indence
                text_y = row_i*self.GDA_block_height + self.GDA_indence
                if index == len(text):
                    break
                else:
                    self.screen_GDA.blit(text[index], [text_x, text_y])


        self.screen.blit(self.screen_GDA, [self.GDA_origin_x, self.GDA_origin_y])

    def draw_robot_data(self):

        self.screen_RDA.fill(GRAY)

        self.RDA_row = 6
        self.RDA_col = 2
        self.RDA_indence = 10
        self.RDA_block_length = (self.RDA_length - 2*self.RDA_indence) / self.RDA_col
        self.RDA_block_height = (self.RDA_height - 2*self.RDA_indence) / self.RDA_row

        text = []
        text.append(font_title.render("Robot Data", True, BLACK))
        text.append(font_title.render("", True, BLACK))
        text.append(font_data.render("Duckiebot ID: " + str(self.RDA_data['bot_id']), True, BLACK))
        text.append(font_data.render("Duckiebot Time: " + str(self.RDA_data['bot_t']), True, BLACK))
        text.append(font_data.render("Position x: " + str(self.RDA_data['x']), True, BLACK))
        text.append(font_data.render("delta x: " + str(self.RDA_data['delta_x']), True, BLACK))
        text.append(font_data.render("Position y: " + str(self.RDA_data['y']), True, BLACK))
        text.append(font_data.render("delta y: "+ str(self.RDA_data['delta_y']), True, BLACK))
        text.append(font_data.render("Orientation theta: " + str(self.RDA_data['theta']), True, BLACK))
        text.append(font_data.render("delta theta: "+ str(self.RDA_data['delta_theta']), True, BLACK))
        text.append(font_data.render("Watchtowers: "+ str(self.RDA_data['watchtower']), True, BLACK))

        row_i = 0
        col_i = 0
        for row_i in range(self.RDA_row):
            for col_i in range(self.RDA_col):
                index = row_i*self.RDA_col + col_i
                text_x = col_i*self.RDA_block_length + self.RDA_indence
                text_y = row_i*self.RDA_block_height + self.RDA_indence
                if index == len(text):
                    break
                else:
                    self.screen_RDA.blit(text[index], [text_x, text_y])

        self.screen.blit(self.screen_RDA, [self.RDA_origin_x, self.RDA_origin_y])

    def draw_duckiebot(self):

        for id_key in self.all_duckiebots:
            pose_x = self.all_duckiebots[id_key]['x']
            pose_delta_x = self.all_duckiebots[id_key]['delta_x']
            pose_y = self.all_duckiebots[id_key]['y']
            pose_delta_y = self.all_duckiebots[id_key]['delta_y']
            pose_theta = self.all_duckiebots[id_key]['theta']
            pose_delta_theta = self.all_duckiebots[id_key]['delta_theta']

            # Draw Duckiebot on the map
            mid_x = pose_x * self.m2p
            mid_y = pose_y * self.m2p
            draw_x = mid_x - int(self.bot_img.get_width()/2)
            draw_y = mid_y - int(self.bot_img.get_width()/2)
            pg.draw.circle(self.screen_Map, DUCKIEBOT_COLOR, [int(mid_x), int(mid_y)], int(duckiebot_size*self.m2p)) # Use a circle to represent Duckiebot
            # self.screen_Map.blit(self.bot_img, [draw_x, draw_y]) # Use a image to represent Duckiebot
            print "size, ", int(duckiebot_size*self.m2p)
            print "draw_x, ", int(mid_x)
            print "draw_y, ", int(mid_y)

            # Draw delta_x, delta_y
            draw_delta_x = int(pose_delta_x * self.m2p)
            draw_delta_y = int(pose_delta_y * self.m2p)
            transparant_screen = pg.Surface((draw_delta_x, draw_delta_y))
            transparant_screen.set_alpha(128)
            transparant_screen.fill(DELTA_XY_COLOR)
            self.screen_Map.blit(transparant_screen, [mid_x-draw_delta_x/2, mid_y-draw_delta_y/2])

            # Draw theta and delta_theta
            draw_radius = self.block_size/3
            draw_theta = math.degrees(pose_theta)
            draw_delta_theta = math.degrees(pose_delta_theta)
            self.screen_Map.blit(self.draw_pie(draw_radius, 2*draw_delta_theta, direction=draw_theta, color=DELTA_THETA_COLOR), [mid_x-draw_radius, mid_y-draw_radius])


        self.screen.blit(self.screen_Map, [self.frame_size, self.frame_size])

    def draw_pie(self, radius, angle, direction=0, color=BLUE):

        direction = 360 - direction
        pil_size = 2*radius

        pil_image = Image.new("RGBA", (pil_size, pil_size))
        pil_draw = ImageDraw.Draw(pil_image)
        pil_draw.pieslice((0, 0, pil_size-1, pil_size-1),  direction-angle/2, direction+angle/2, fill=color)

        mode = pil_image.mode
        size = pil_image.size
        data = pil_image.tobytes()

        return pg.image.fromstring(data, size, mode)

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
        self.map_watchtowers = map_data['watchtowers']

        return self.map_tiles

    def quit_program(self):

        pg.quit()
        sys.exit()


if __name__ == '__main__':
    rospy.init_node('map_description_node', anonymous=False)
    node = map_description()
    rospy.spin()
