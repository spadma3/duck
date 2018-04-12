# This script is used to start localization nodes on camera towers in side Duckietown

# source environment
source ~/duckietown/environment.sh

# Make sure that the master of each camera is your server computer
# change "duckietown1" to your server computer
source ~/duckietown/set_ros_master.sh duckietown1

# launch camera node
roslaunch duckietown camera.launch veh:=$HOSTNAME

