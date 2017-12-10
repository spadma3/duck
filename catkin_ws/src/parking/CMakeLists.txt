cmake_minimum_required(VERSION 2.8.3)
project(parking)

find_package(catkin_simple REQUIRED COMPONENTS
        rospy
        roscpp
        genmsg
        geometry_msgs
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
        )

catkin_package()
catkin_simple()

cs_install()
cs_export()

add_executable(image_undistorter src/image_undistorter.cpp)
target_link_libraries(image_undistorter ${catkin_LIBRARIES})



include_directories(
        ${catkin_INCLUDE_DIRS}
)
