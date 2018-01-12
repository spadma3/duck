cmake_minimum_required(VERSION 2.8.3)
project(fleet_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  duckietown_msgs
  rostest

## -- UNCOMMENT BELOW LINE FOR LAPTOP BUILD -- ##
  #rqt_gui_py
)

add_rostest(test/taxi_central.test)
add_rostest(test/message_serialization.test)

catkin_python_setup()


add_service_files(
  FILES
  GraphSearch.srv
  VirtualDuckiebotLocation.srv
  VirtualDuckiebot.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  duckietown_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fleet_planning
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
  scripts/rqt_fleet_planning
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
