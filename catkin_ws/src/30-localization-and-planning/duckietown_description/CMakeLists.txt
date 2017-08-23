cmake_minimum_required(VERSION 2.8.3)
project(duckietown_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/duckietown_description_tester_node.test)
    add_rostest(tests/map_generation_tester.test)
endif()
