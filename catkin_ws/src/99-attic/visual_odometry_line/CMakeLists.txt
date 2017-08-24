cmake_minimum_required(VERSION 2.8.3)
project(visual_odometry_line)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  cv_bridge
  tf
)

catkin_python_setup()



catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
