#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Path planing algorithm
# Version 2
# slow, colours are right
# Samuel Nyffenegger

import sys, os, random, math, pygame
import numpy as np
from pygame.locals import *
from math import sqrt,cos,sin,atan2

"""
Global variables
"""
# parking lot parameters
lot_width = 2*585               # mm, lot = 2x2 squares
lot_height = 2*585              # mm
wide_tape_width = 50            # mm, red, white
narrow_tape_width = 25          # mm, yellow
april_tag_basement_length = 50  # mm,
april_tag_screen_length = 80    # mm
space_length = 270              # mm from border, without april tag
lanes_length = 310               # mm at entrance, exit

# programming parameters
figure_scaling = 0.5;

# key: object on map <--> number
map_keys = {
    "empty": 0,
        "white lane": 1,
        "yellow lane": 2,
        "red lane": 3,
        "april tag": 4,
        "parked DB": 5,
}

# colours
red = 255, 0, 0
white = 240, 240, 240
black = 0, 0, 0
yellow = 255, 255, 0
background = 20,20,20


"""
Functions
"""
# draws the map given the parking matrix
def draw_parking_map(screen, map):
    lot_height, lot_width = map.shape
    for h in range(lot_height):
        for w in range(lot_width):
            if map[h,w] == map_keys["red lane"]:
                pygame.draw.rect(screen,red, [w*figure_scaling, h*figure_scaling, 1*figure_scaling, 1*figure_scaling])
            elif map[h,w] == map_keys["yellow lane"]:
                pygame.draw.rect(screen,yellow, [w*figure_scaling, h*figure_scaling, 1*figure_scaling, 1*figure_scaling])
            elif map[h,w] == map_keys["white lane"]:
                pygame.draw.rect(screen,white, [w*figure_scaling, h*figure_scaling, 1*figure_scaling, 1*figure_scaling])
            else:
                pygame.draw.rect(screen,black, [w*figure_scaling, h*figure_scaling, 1*figure_scaling, 1*figure_scaling])




# helper function to draw an arrow in the map
def plot_pose_as_arrow(pose, colour="k", arrow_magnitude=75):
    # this function swaps x,y in pose to generate the arrow
    plt.arrow( pose[1], pose[0], arrow_magnitude*np.cos(-pose[2]), arrow_magnitude*np.sin(-pose[2]), fc=colour, ec=colour, head_width=arrow_magnitude/5.0, head_length=arrow_magnitude/5.0)

# pose assigenment: entrance, parking space, exit
def pose_from_key(key):
    if key == "entrance" or key == 0:
        return np.array([lanes_length/2.0, wide_tape_width+length_red_line/2.0, -math.pi/2.0])
    elif key == "space 1" or key == 1:
        return np.array([lot_height-space_length/2.0, lot_width/8.0, -math.pi/2.0])
    elif key == "space 2" or key == 2:
        return np.array([lot_height-space_length/2.0, lot_width/8.0 + lot_width/4.0, -math.pi/2.0])
    elif key == "space 3" or key == 3:
        return np.array([lot_height-space_length/2.0, lot_width/8.0 + 2*lot_width/4.0, -math.pi/2.0])
    elif key == "space 4" or key == 4:
        return np.array([lot_height-space_length/2.0, lot_width/8.0 + 3*lot_width/4.0, -math.pi/2.0])
    elif key == "space 5" or key == 5:
        return np.array([space_length/2.0, lot_width/8.0 + 2*lot_width/4.0, math.pi/2.0])
    elif key == "space 6" or key == 6:
        return np.array([space_length/2.0, lot_width/8.0 + 3*lot_width/4.0, math.pi/2.0])
    elif key == "exit" or key == 7:
        return np.array([lanes_length/2.0, wide_tape_width+narrow_tape_width+3.0/2.0*length_red_line, math.pi/2.0])
    else:
        print("parking space not found")
        exit(1)


def main():

    """
    Static map generation
    """
    ### control parameters
    plot_static_map = True

    # init screen
    pygame.init()
    screen = pygame.display.set_mode([int(figure_scaling*lot_height), int(figure_scaling*lot_height)])
    pygame.display.set_caption('Parking map')
    screen.fill(background)

    ### build map
    # empty map
    map = np.empty((lot_height,lot_width))

    # red stopping lines
    length_red_line = int( (lot_width/2.0 - 2.0*wide_tape_width - 1.0*narrow_tape_width) / 2.0 )
    map[lanes_length-wide_tape_width:lanes_length,wide_tape_width:wide_tape_width+length_red_line] = map_keys["red lane"]
    map[0:wide_tape_width,wide_tape_width+length_red_line+narrow_tape_width:int(lot_width/2.0-wide_tape_width)] = map_keys["red lane"]

    # white lanes
    map[0:lanes_length,0:wide_tape_width] = map_keys["white lane"]
    map[0:lanes_length,int(lot_width/2.0-wide_tape_width):int(lot_width/2.0)] = map_keys["white lane"]

    # yellow lanes
    map[0:lanes_length,wide_tape_width+length_red_line:wide_tape_width+length_red_line+narrow_tape_width] = map_keys["yellow lane"]
    map[lot_height-space_length:lot_height,0:narrow_tape_width] = map_keys["yellow lane"]
    map[lot_height-space_length:lot_height,int(lot_height/4.0-narrow_tape_width/2.0):int(lot_height/4.0+narrow_tape_width/2.0)] = map_keys["yellow lane"]
    map[lot_height-space_length:lot_height,int(lot_height/2.0-narrow_tape_width/2.0):int(lot_height/2.0+narrow_tape_width/2.0)] = map_keys["yellow lane"]
    map[lot_height-space_length:lot_height,int(lot_height*3.0/4.0-narrow_tape_width/2.0):int(lot_height*3.0/4.0+narrow_tape_width/2.0)] = map_keys["yellow lane"]
    map[lot_height-space_length:lot_height,lot_width-narrow_tape_width:lot_width] = map_keys["yellow lane"]
    map[0:space_length,int(lot_height*3.0/4.0-narrow_tape_width/2.0):int(lot_height*3.0/4.0+narrow_tape_width/2.0)] = map_keys["yellow lane"]

    # draw map
    draw_parking_map(screen, map)
    pygame.display.update()



if __name__ == '__main__':
    main()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

