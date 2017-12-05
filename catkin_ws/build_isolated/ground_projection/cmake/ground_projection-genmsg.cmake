# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ground_projection: 0 messages, 3 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iduckietown_msgs:/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ground_projection_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetImageCoord.srv" NAME_WE)
add_custom_target(_ground_projection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ground_projection" "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetImageCoord.srv" "duckietown_msgs/Vector2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/EstimateHomography.srv" NAME_WE)
add_custom_target(_ground_projection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ground_projection" "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/EstimateHomography.srv" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetGroundCoord.srv" NAME_WE)
add_custom_target(_ground_projection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ground_projection" "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetGroundCoord.srv" "duckietown_msgs/Vector2D:geometry_msgs/Point"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ground_projection
  "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetImageCoord.srv"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ground_projection
)
_generate_srv_cpp(ground_projection
  "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/EstimateHomography.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ground_projection
)
_generate_srv_cpp(ground_projection
  "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetGroundCoord.srv"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ground_projection
)

### Generating Module File
_generate_module_cpp(ground_projection
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ground_projection
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ground_projection_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ground_projection_generate_messages ground_projection_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetImageCoord.srv" NAME_WE)
add_dependencies(ground_projection_generate_messages_cpp _ground_projection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/EstimateHomography.srv" NAME_WE)
add_dependencies(ground_projection_generate_messages_cpp _ground_projection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetGroundCoord.srv" NAME_WE)
add_dependencies(ground_projection_generate_messages_cpp _ground_projection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ground_projection_gencpp)
add_dependencies(ground_projection_gencpp ground_projection_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ground_projection_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ground_projection
  "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetImageCoord.srv"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection
)
_generate_srv_py(ground_projection
  "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/EstimateHomography.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection
)
_generate_srv_py(ground_projection
  "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetGroundCoord.srv"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection
)

### Generating Module File
_generate_module_py(ground_projection
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ground_projection_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ground_projection_generate_messages ground_projection_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetImageCoord.srv" NAME_WE)
add_dependencies(ground_projection_generate_messages_py _ground_projection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/EstimateHomography.srv" NAME_WE)
add_dependencies(ground_projection_generate_messages_py _ground_projection_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/10-lane-control/ground_projection/srv/GetGroundCoord.srv" NAME_WE)
add_dependencies(ground_projection_generate_messages_py _ground_projection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ground_projection_genpy)
add_dependencies(ground_projection_genpy ground_projection_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ground_projection_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ground_projection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ground_projection
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ground_projection_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ground_projection_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ground_projection_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET duckietown_msgs_generate_messages_cpp)
  add_dependencies(ground_projection_generate_messages_cpp duckietown_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ground_projection/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ground_projection_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ground_projection_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ground_projection_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET duckietown_msgs_generate_messages_py)
  add_dependencies(ground_projection_generate_messages_py duckietown_msgs_generate_messages_py)
endif()
