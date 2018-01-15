#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Path planning for duckietown parking_space
Samuel Nyffenegger
"""

from parking_main import *

def blank_map(start_number=None, end_number=None):
    """
    Problem definition and heuristics
    """
    start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number = initialize(start_number, end_number)
    objects = define_objects()
    obstacles = define_obstacles(objects)

    # do standard plotting
    px, py, found_path = [], [], False
    do_plotting(start_x, start_y, start_yaw, start_number, end_x, end_y, end_yaw, end_number, px, py, objects, obstacles, found_path);

    # additional plotting
    start_x, start_y, start_yaw = pose_from_key(0)
    plt.text(int(start_x), int(start_y)+20, "entrance\n(0)",fontsize=10, horizontalalignment='center', color="w" )
    plt.text(int(start_x), int(start_y)-220, "start pose",fontsize=10, horizontalalignment='center', color="r" )
    start_x, start_y, start_yaw = pose_from_key(7)
    plt.text(int(start_x), int(start_y)+20, "exit\n(7)",fontsize=10, horizontalalignment='center', color="w" )
    for i in range(6):
        start_x, start_y, start_yaw = pose_from_key(i+1)
        plt.text(int(start_x), int(start_y)+20, "space {}".format(i+1),fontsize=10, horizontalalignment='center', color="w" )
    plt.annotate('obstacles', xy=(lot_width/2.0+50, lot_width/2.0+50), xytext=(lot_width/2.0+150,lot_width/2.0+100), arrowprops=dict(fc='w', ec='w', shrink=0.03),color='w', fontsize=10)
    plt.annotate('obstacles', xy=(lot_width/2.0-wide_tape_width+wide_tape_width, lot_height-lanes_length), xytext=(lot_width/2.0+150,lot_width/2.0+100), arrowprops=dict(fc='w', ec='w', shrink=0.03),color='w', fontsize=10)



    start_x, start_y, start_yaw = pose_from_key(4)
    plt.text(int(start_x)+10, int(start_y)-100, "end pose",fontsize=10, horizontalalignment='center', color="g" )



    # save
    plt.savefig('images/map_{}_{}.pdf'.format(start_number,end_number))
    plt.pause(1)
    plt.show()
"""
main file
"""
if __name__ == '__main__':

    # path calculation
    init()
    start_numbers = [0]
    end_numbers = [4]
    for start_number, end_number in zip(start_numbers, end_numbers):
        print("Planning a path from {} to {}: ".format(start_number, end_number))
        blank_map(start_number, end_number)
        print("\n")
