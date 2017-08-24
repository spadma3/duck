cmake_minimum_required(VERSION 2.8.3)
project(bag_stamper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
