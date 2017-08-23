cmake_minimum_required(VERSION 2.8.3)
project(parallel_autonomy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  geometry_msgs
  std_msgs
)

catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
