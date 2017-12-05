# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(rgb_led_CONFIG_INCLUDED)
  return()
endif()
set(rgb_led_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(rgb_led_SOURCE_PREFIX /home/yunfantang/duckietown/catkin_ws/src/40-coordination/rgb_led)
  set(rgb_led_DEVEL_PREFIX /home/yunfantang/duckietown/catkin_ws/devel_isolated/rgb_led)
  set(rgb_led_INSTALL_PREFIX "")
  set(rgb_led_PREFIX ${rgb_led_DEVEL_PREFIX})
else()
  set(rgb_led_SOURCE_PREFIX "")
  set(rgb_led_DEVEL_PREFIX "")
  set(rgb_led_INSTALL_PREFIX /home/yunfantang/duckietown/catkin_ws/install_isolated)
  set(rgb_led_PREFIX ${rgb_led_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'rgb_led' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(rgb_led_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(rgb_led_INCLUDE_DIRS "")
  set(_include_dirs "")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${rgb_led_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'rgb_led' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'Mack <mack@duckietown.org>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'rgb_led' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/yunfantang/duckietown/catkin_ws/install_isolated/${idir}'.  Ask the maintainer 'Mack <mack@duckietown.org>' to fix it.")
    endif()
    _list_append_unique(rgb_led_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND rgb_led_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND rgb_led_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND rgb_led_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/yunfantang/duckietown/catkin_ws/install_isolated/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dagu_car/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/pkg_name/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/pi_camera/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/parallel_autonomy/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/navigation/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/mdoap/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/localization/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/line_detector2/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/line_detector/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/led_joy_mapper/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/led_interpreter/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/led_detection/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/lane_filter/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/lane_control/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/joy_mapper/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/intersection_control/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/indefinite_navigation/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/ground_projection/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/fsm/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/easy_regression/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/easy_node/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/easy_logs/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/easy_algo/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_unit_test/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckiebot_visualizer/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckie_rr_bridge/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dt_filtering_swaggyduck/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dt_augmented_reality_swaggyduck/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/dt-live-instagram_swaggyduck/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/complete_image_pipeline/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/anti_instagram/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_msgs/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_logs/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_description/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_demos/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckieteam/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/apriltags/lib;/home/yunfantang/duckietown/catkin_ws/devel_isolated/adafruit_drivers/lib;/home/yunfantang/duckietown/catkin_ws/devel/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(rgb_led_LIBRARY_DIRS ${lib_path})
      list(APPEND rgb_led_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'rgb_led'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND rgb_led_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(rgb_led_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${rgb_led_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 rgb_led_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${rgb_led_dep}_FOUND)
      find_package(${rgb_led_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${rgb_led_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(rgb_led_INCLUDE_DIRS ${${rgb_led_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(rgb_led_LIBRARIES ${rgb_led_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${rgb_led_dep}_LIBRARIES})
  _list_append_deduplicate(rgb_led_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(rgb_led_LIBRARIES ${rgb_led_LIBRARIES})

  _list_append_unique(rgb_led_LIBRARY_DIRS ${${rgb_led_dep}_LIBRARY_DIRS})
  list(APPEND rgb_led_EXPORTED_TARGETS ${${rgb_led_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${rgb_led_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
