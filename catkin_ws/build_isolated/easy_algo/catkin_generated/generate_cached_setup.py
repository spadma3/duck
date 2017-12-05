# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_unit_test;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckiebot_visualizer;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckie_rr_bridge;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dt_filtering_swaggyduck;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dt_augmented_reality_swaggyduck;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dt-live-instagram_swaggyduck;/home/yunfantang/duckietown/catkin_ws/devel_isolated/complete_image_pipeline;/home/yunfantang/duckietown/catkin_ws/devel_isolated/anti_instagram;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_msgs;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_logs;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_description;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_demos;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckieteam;/home/yunfantang/duckietown/catkin_ws/devel_isolated/apriltags;/home/yunfantang/duckietown/catkin_ws/devel_isolated/adafruit_drivers;/home/yunfantang/duckietown/catkin_ws/devel;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/yunfantang/duckietown/catkin_ws/devel_isolated/easy_algo/env.sh')

output_filename = '/home/yunfantang/duckietown/catkin_ws/build_isolated/easy_algo/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
