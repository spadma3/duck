# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "duckietown_msgs: 43 messages, 2 services")

set(MSG_I_FLAGS "-Iduckietown_msgs:/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(duckietown_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg" "geometry_msgs/Point:duckietown_msgs/ObstacleType:std_msgs/Header:duckietown_msgs/ObstacleProjectedDetection"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg" "geometry_msgs/Vector3Stamped:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg" "duckietown_msgs/Vector2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg" "geometry_msgs/Point:duckietown_msgs/ObstacleType"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg" "duckietown_msgs/Rect:duckietown_msgs/ObstacleType:std_msgs/Header:duckietown_msgs/ObstacleImageDetection"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:duckietown_msgs/AprilTagDetection:geometry_msgs/Point:geometry_msgs/Pose:duckietown_msgs/TagInfo"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg" "std_msgs/Float32:std_msgs/Bool:std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg" "duckietown_msgs/Rect"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg" "duckietown_msgs/Vector2D:duckietown_msgs/Segment:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg" "std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg" "std_msgs/Bool:geometry_msgs/Point32:std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg" "duckietown_msgs/Rect:sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:duckietown_msgs/AprilTagDetection:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg" "duckietown_msgs/Vector2D"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg" "duckietown_msgs/LEDDetection:duckietown_msgs/LEDDetectionArray:duckietown_msgs/Vector2D:sensor_msgs/CompressedImage:std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg" "duckietown_msgs/Rect:duckietown_msgs/ObstacleType"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg" "duckietown_msgs/LEDDetection:duckietown_msgs/Vector2D"
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg" ""
)

get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg" NAME_WE)
add_custom_target(_duckietown_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "duckietown_msgs" "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)

### Generating Services
_generate_srv_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)
_generate_srv_cpp(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
)

### Generating Module File
_generate_module_cpp(duckietown_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(duckietown_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(duckietown_msgs_generate_messages duckietown_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_cpp _duckietown_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(duckietown_msgs_gencpp)
add_dependencies(duckietown_msgs_gencpp duckietown_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS duckietown_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3Stamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/CompressedImage.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float32.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg;/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_msg_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)

### Generating Services
_generate_srv_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)
_generate_srv_py(duckietown_msgs
  "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
)

### Generating Module File
_generate_module_py(duckietown_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(duckietown_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(duckietown_msgs_generate_messages duckietown_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetectionList.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramTransform.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vsample.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Trajectory.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetValue.srv" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/srv/SetFSMState.srv" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ThetaDotSample.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Segment.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SourceTargetNodes.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsParameters.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rect.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CarControl.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDInterpreter.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Vector2D.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmdStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleProjectedDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/TagInfo.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationClearance.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pose2DStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetectionList.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagsWithInfos.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleType.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AntiInstagramHealth.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/BoolStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/FSMState.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SignalsDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/WheelsCmd.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehiclePose.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Rects.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SegmentList.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/StopLineReading.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/VehicleCorners.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/CoordinationSignal.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/SceneSegments.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/AprilTagDetectionArray.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Twist2DStamped.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/Pixel.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionDebugInfo.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LanePose.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/ObstacleImageDetection.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/LEDDetectionArray.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/DuckieSensor.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yunfantang/duckietown/catkin_ws/src/00-infrastructure/duckietown_msgs/msg/KinematicsWeights.msg" NAME_WE)
add_dependencies(duckietown_msgs_generate_messages_py _duckietown_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(duckietown_msgs_genpy)
add_dependencies(duckietown_msgs_genpy duckietown_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS duckietown_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/duckietown_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(duckietown_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(duckietown_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(duckietown_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/duckietown_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(duckietown_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(duckietown_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(duckietown_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
