cmake_minimum_required(VERSION 2.8.3)
project(scene_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  duckietown_msgs
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)


add_service_files(
  FILES
  SegmentImage.srv
)
 
generate_messages(
  DEPENDENCIES
  # std_msgs
  sensor_msgs
  # geometry_msgs
  duckietown_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene_segmentation
 CATKIN_DEPENDS cv_bridge duckietown_msgs geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(scene_segmentation
  src/scene_segmentation.cpp
  src/egbis/segment-image.cpp
  src/egbis/segment-graph.cpp
  src/egbis/disjoint-set.cpp
  src/egbis/filter.cpp
  src/egbis/misc.cpp
)

# src/egbis/segment-image.h
# src/egbis/segment-graph.h
# src/egbis/disjoint-set.h
# src/egbis/image.h
# src/egbis/filter.h
# src/egbis/misc.h

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(scene_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scene_segmentation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
