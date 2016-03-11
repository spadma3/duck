cmake_minimum_required(VERSION 2.8.3)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages should use this.
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(GTSAM REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
message(STATUS "Found GTSAM")

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(sum_and_average_node src/sum_and_average_node.cpp)
target_link_libraries(sum_and_average_node ${catkin_LIBRARIES})

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node ${catkin_LIBRARIES})
target_link_libraries(slam_node ${GTSAM_LIBRARIES})
