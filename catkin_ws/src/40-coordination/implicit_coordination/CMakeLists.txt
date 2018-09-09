cmake_minimum_required(VERSION 2.8.3)
project(implicit_coordination)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages must use this.
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg_name
#  CATKIN_DEPENDS duckietown_msgs roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
