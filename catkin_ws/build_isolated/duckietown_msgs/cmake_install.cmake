# Install script for directory: /home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yunfantang/duckietown/catkin_ws/install_isolated")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yunfantang/duckietown/catkin_ws/install_isolated/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yunfantang/duckietown/catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yunfantang/duckietown/catkin_ws/install_isolated/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yunfantang/duckietown/catkin_ws/install_isolated" TYPE PROGRAM FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yunfantang/duckietown/catkin_ws/install_isolated/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yunfantang/duckietown/catkin_ws/install_isolated" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/setup.bash")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yunfantang/duckietown/catkin_ws/install_isolated/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yunfantang/duckietown/catkin_ws/install_isolated" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/setup.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yunfantang/duckietown/catkin_ws/install_isolated/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yunfantang/duckietown/catkin_ws/install_isolated" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/setup.zsh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/yunfantang/duckietown/catkin_ws/install_isolated/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/yunfantang/duckietown/catkin_ws/install_isolated" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/msg" TYPE FILE FILES
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/srv" TYPE FILE FILES
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv"
    "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/cmake" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/duckietown_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_msgs/include/duckietown_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_msgs/lib/python2.7/dist-packages/duckietown_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/yunfantang/duckietown/catkin_ws/devel_isolated/duckietown_msgs/lib/python2.7/dist-packages/duckietown_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/duckietown_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/cmake" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/duckietown_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs/cmake" TYPE FILE FILES
    "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/duckietown_msgsConfig.cmake"
    "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/catkin_generated/installspace/duckietown_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/duckietown_msgs" TYPE FILE FILES "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/yunfantang/duckietown/catkin_ws/build_isolated/duckietown_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
