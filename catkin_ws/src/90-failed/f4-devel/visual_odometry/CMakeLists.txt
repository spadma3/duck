cmake_minimum_required(VERSION 2.8.3)
project(visual_odometry)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/odometry_training_pairs_tester_node.test)
    add_rostest(tests/visual_odometry_april_tags_tester_node.test)
endif()
