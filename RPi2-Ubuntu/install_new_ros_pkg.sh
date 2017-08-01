# Install and build ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm geometry_msgs --rosdistro $ROS_DISTRO --deps --wet-only --exclude roslisp --tar > $ROS_DISTRO-custom_ros.rosinstall

#
wstool merge -t src $ROS_DISTRO-custom_ros.rosinstall
wstool update -t src