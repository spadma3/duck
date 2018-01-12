cmake_minimum_required(VERSION 2.8.3)
project(fleet_messaging)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs # Every duckietown packages must use this.
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg_name
#  CATKIN_DEPENDS duckietown_msgs roscpp rospy
#  DEPENDS system_lib
)


# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



