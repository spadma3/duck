cmake_minimum_required(VERSION 2.8.3)
project(rgb_led)

find_package(catkin REQUIRED COMPONENTS
	rospy
	duckietown_msgs # Every duckietown packages should use this.
	std_msgs
	message_generation
)

catkin_python_setup()



add_service_files(
  FILES
  PlayLEDPattern.srv
)
generate_messages()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
