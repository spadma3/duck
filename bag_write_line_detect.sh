#!/bin/bash

cd ~/duckietown

export VEH=$1
export HUE=$2
export SAT=$3


source environment.sh
source set_ros_master.sh $VEH


rosbag record -O /home/bings/Documents/Tests/Line_Detector/test1_29.06.18/h_$2_s_$3_$VEH.bag --node=/$VEH/line_detector_node --duration=5s
