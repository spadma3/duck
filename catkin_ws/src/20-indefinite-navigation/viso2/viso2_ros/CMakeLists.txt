cmake_minimum_required(VERSION 2.8.3)
project(viso2_ros)

find_package(catkin REQUIRED COMPONENTS
  libviso2
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  image_transport
  cv_bridge
  image_geometry
  tf
  pcl_ros
  std_srvs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

#option(ARM_CROSS_COMPILATION "ARM Cross Compilation" ON)

add_message_files( DIRECTORY msg FILES VisoInfo.msg)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package()

#add_definitions(-msse3)
#if(ARM_CROSS_COMPILATION)
SET(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CXX_FLAGS -mfpu=neon)
#endif(ARM_CROSS_COMPILATION)

##set(NEON_FLAGS "-DENABLE_NEON -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a1$)
##set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wno-format-security ${NEON_$})
##set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NEON_FLAGS}")

include_directories(src ${libviso2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(stereo_odometer
  src/stereo_odometer.cpp)
add_dependencies(stereo_odometer ${catkin_EXPORTED_TARGETS})
add_dependencies(stereo_odometer ${PROJECT_NAME}_gencpp)

add_executable(mono_odometer
  src/mono_odometer.cpp)
add_dependencies(mono_odometer ${catkin_EXPORTED_TARGETS})
add_dependencies(mono_odometer ${PROJECT_NAME}_gencpp)

target_link_libraries(stereo_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
find_package(Boost REQUIRED COMPONENTS signals thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(stereo_odometer ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mono_odometer ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})



