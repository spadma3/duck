cmake_minimum_required(VERSION 2.8.3)
project(multivehicle_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages must use this.
  std_msgs
  sensor_msgs
  message_generation
)

catkin_python_setup()


add_message_files(
  FILES
  Tracklet.msg
  TrackletList.msg
)

generate_messages(
  DEPENDENCIES
  duckietown_msgs # Every duckietown packages must use this.
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
