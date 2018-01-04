cmake_minimum_required(VERSION 2.8.3)
project(parking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

 add_message_files
 (
   FILES
   Reference_for_control.msg
   Pose_duckiebot.msg
 )

 generate_messages
 (
   DEPENDENCIES
   std_msgs
 )

catkin_package
(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
