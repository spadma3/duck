# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "navigation: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iduckietown_msgs:/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/navigation/srv/GraphSearch.srv" NAME_WE)
add_custom_target(_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation" "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/navigation/srv/GraphSearch.srv" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(navigation
  "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/navigation/srv/GraphSearch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation
)

### Generating Module File
_generate_module_cpp(navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(navigation_generate_messages navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/navigation/srv/GraphSearch.srv" NAME_WE)
add_dependencies(navigation_generate_messages_cpp _navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_gencpp)
add_dependencies(navigation_gencpp navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(navigation
  "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/navigation/srv/GraphSearch.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation
)

### Generating Module File
_generate_module_py(navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(navigation_generate_messages navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/20-indefinite-navigation/navigation/srv/GraphSearch.srv" NAME_WE)
add_dependencies(navigation_generate_messages_py _navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_genpy)
add_dependencies(navigation_genpy navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET duckietown_msgs_generate_messages_cpp)
  add_dependencies(navigation_generate_messages_cpp duckietown_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(navigation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET duckietown_msgs_generate_messages_py)
  add_dependencies(navigation_generate_messages_py duckietown_msgs_generate_messages_py)
endif()
