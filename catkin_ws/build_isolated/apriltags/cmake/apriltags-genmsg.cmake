# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "apriltags: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iapriltags:/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(apriltags_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetections.msg" NAME_WE)
add_custom_target(_apriltags_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "apriltags" "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetections.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point32:geometry_msgs/Point:apriltags/AprilTagDetection:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg" NAME_WE)
add_custom_target(_apriltags_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "apriltags" "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg" "geometry_msgs/Quaternion:geometry_msgs/Point32:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(apriltags
  "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags
)
_generate_msg_cpp(apriltags
  "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags
)

### Generating Services

### Generating Module File
_generate_module_cpp(apriltags
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(apriltags_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(apriltags_generate_messages apriltags_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_generate_messages_cpp _apriltags_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(apriltags_generate_messages_cpp _apriltags_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_gencpp)
add_dependencies(apriltags_gencpp apriltags_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(apriltags
  "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetections.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags
)
_generate_msg_py(apriltags
  "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags
)

### Generating Services

### Generating Module File
_generate_module_py(apriltags
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(apriltags_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(apriltags_generate_messages apriltags_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetections.msg" NAME_WE)
add_dependencies(apriltags_generate_messages_py _apriltags_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(apriltags_generate_messages_py _apriltags_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltags_genpy)
add_dependencies(apriltags_genpy apriltags_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltags_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltags
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(apriltags_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(apriltags_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltags
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(apriltags_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(apriltags_generate_messages_py geometry_msgs_generate_messages_py)
endif()
