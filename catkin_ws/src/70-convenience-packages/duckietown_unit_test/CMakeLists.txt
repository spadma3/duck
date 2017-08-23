cmake_minimum_required(VERSION 2.8.3)
project(duckietown_unit_test)

find_package(catkin REQUIRED COMPONENTS
  # roscpp
  # rospy
  # duckietown_msgs # Every duckietown_unit_test packages should use this.
)


catkin_package( )

include_directories(
  ${catkin_INCLUDE_DIRS}
)
