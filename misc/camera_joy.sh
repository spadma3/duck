#!/bin/sh
rosrun image_view image_view image:="/$1/camera_node/image" _image_transport:=compressed &
export PID_RQT=$!
source environment.#!/bin/sh
source set_ros_master.sh $1
source set_vehicle_name.sh $1
export VEHICLE_NAME=$1
python misc/virtualJoy/virtualJoy.py &
PID_JOY=$!
sleep 2
clear
echo "Press any key to exit..."
read
kill -KILL $PID_JOY
kill -KILL $PID_RQT
