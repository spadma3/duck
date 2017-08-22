cmake_minimum_required(VERSION 2.8.3)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(sum_and_average_node src/sum_and_average_node.cpp)
target_link_libraries(sum_and_average_node ${catkin_LIBRARIES})

add_executable(slamNode src/slamNode.cpp)
target_link_libraries(slamNode ${catkin_LIBRARIES})
