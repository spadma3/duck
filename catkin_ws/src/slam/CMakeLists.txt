cmake_minimum_required(VERSION 2.8.3)
project(slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  std_msgs
  visualization_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/include/eigen3) # manually include eigen header path
# include_directories(/usr/include/tbb) 

catkin_package(
#   INCLUDE_DIRS include
   CATKIN_DEPENDS duckietown_msgs roscpp rospy
   DEPENDS gtsam
)

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node 
	${catkin_LIBRARIES}
	gtsam)

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)

#add_executable(sum_and_average_node src/sum_and_average_node.cpp)
#target_link_libraries(sum_and_average_node ${catkin_LIBRARIES})