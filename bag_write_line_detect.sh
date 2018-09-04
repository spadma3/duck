#!/bin/bash

#cd ~/duckietown

#export VEH=$1
#export HUE=$2
#export SAT=$3
#export BRI=$2


#source environment.sh
#source set_ros_master.sh megabot03

i="0"

while [ $i -lt 42 ]
do
read -p "Press any key to continue... " -n1 -s
printf "$i"
roslaunch line_detector line_detector_bag.launch bagin:=/home/megaduck/Tests/illumination_line_detect_"$i".bag bagout:=/home/megaduck/Testresults/illumination_line_detect_"$i"_result.bag veh:=megabot03
#rosbag record -O /home/duckietown/Documents/Tests/illumination_line_detect_"$i".bag --node=/megabot03/line_detector_node --duration=5s
#rosbag record -O /home/duckietown/Documents/Tests/illumination_lane_filter_"$i".bag --node=/megabot03/lane_filter_node --duration=5s
i=$[$i+1]
done
