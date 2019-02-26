demos:
	@echo "$(sep)Demos"
	@echo
	@echo "These are simple demos"
	@echo
	@echo TODO: to write
	@echo

### Intersection navigation demo
demo-intersection-navigation: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch intersection_navigation intersection_navigation_node.launch"

### These are not using master.launch
demo-joystick: check-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick.launch veh:=$(vehicle_name)"

demo-joystick-high-speed: check-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick.launch veh:=$(vehicle_name) joy_mapper_param_file_name:=high_speed "

demo-joystick-camera: check-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name)"

demo-joystick-camera-high-speed: check-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name) joy_mapper_param_file_name:=high_speed "

demo-line_detector: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name)"

demo-joystick-perception: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos master.launch fsm_file_name:=joystick"

demo-lane_following-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos lane_following.launch line_detector_param_file_name:=$*"

demo-led-fancy1: check-environment
	bash -c "source environment.sh; rosrun rgb_led fancy1"

demo-led-fancy2: check-environment
	bash -c "source environment.sh; rosrun rgb_led fancy2"

demo-led-blink-%: check-environment
	bash -c "source environment.sh; rosrun rgb_led blink $*"

demo-implicit_coordination: check-environment
	bash -c "source environment.sh; source set_vehicle_name.sh; roslaunch duckietown_demos implicit_coordination.launch"
demo-line_detector-default:     demo-line_detector-quiet-default
demo-line_detector-guy:         demo-line_detector-quiet-guy
demo-line_detector-universal:   demo-line_detector-quiet-universal
demo-line_detector-default_ld2: demo-line_detector-quiet-default_ld2

demo-line_detector-quiet-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name) line_detector_param_file_name:=$* verbose:=false"

demo-vehicle_follow_leader: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid.launch"


# Basic demos
# traffic lights
traffic-light: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch traffic_light traffic_light_node.launch veh:=$(vehicle_name)"

# auto_localization demo
auto_localization_watchtower: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown auto_localization_watchtower.launch veh:=$(vehicle_name) param_file_name:=autolocal IP:=$(IP)"
auto_localization_laptop: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown auto_localization_server.launch veh:=$(vehicle_name) param_file_name:=autolocal IP:=$(IP) map:=$(map)"
auto_localization_server: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos tcp_server.launch param_file_name:=autolocal IP:=$(IP)"
# auto localization with same master (master on the laptop)
auto_localization_watchtower_same_master_%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh; roslaunch duckietown auto_localization_watchtower.launch veh:=$(vehicle_name) same_master:=true param_file_name:=autolocal"
auto_localization_laptop_same_master: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown auto_localization_server.launch veh:=$(vehicle_name) same_master:=true param_file_name:=autolocal map:=$(map)"

# auto_localization system calibration
auto_localization_calibration_watchtower: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown auto_localization_watchtower_calibration.launch veh:=$(vehicle_name) param_file_name:=autolocal_calibration IP:=$(IP)"

auto_localization_calibration_laptop: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown auto_localization_server_calibration.launch veh:=$(vehicle_name) param_file_name:=autolocal_calibration IP:=$(IP)  map:=$(map)"
auto_localization_calibration_server: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos tcp_server.launch param_file_name:=autolocal_calibration IP:=$(IP)"

turn-duration-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*; rosparam set /$*/open_loop_intersection_control_node/turn_$(type) '[[0.8, 0.43, 0],[$(time), 0.43, 2.896],[0.8, 0.43, 0.0]]'; "
