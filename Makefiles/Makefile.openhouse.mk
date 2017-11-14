
openhouse:
	@echo "$(sep)Open house demos"
	@echo
	@echo "These were the open house demos."
	@echo
	@echo TODO: to write
	@echo


openhouse-dp1: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch"

demo-lane-following: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos lane_following.launch"

openhouse-dp1-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch line_detector_param_file_name:=$*"

openhouse-dp1-veh: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch vehicle_avoidance:=true"

openhouse-dp2: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_vehicle_avoid.launch"

openhouse-dp2-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_vehicle_avoid.launch line_detector_param_file_name:=$*"

openhouse-dp2-vehicle: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid.launch"

openhouse-dp2-obstacle: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_avoid.launch"

openhouse-dp2-vehicle-no-wheels: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid_nowheels.launch"

openhouse-dp2-obstacle-no-wheels: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_avoid_nowheels.launch"

openhouse-dp6a-generate-map-%: check-environment
	bash -c 'echo -n "Enter full path to the tags .csv file: "; read tag_map; echo -n "Enter full path to the tiles .csv file: "; read tile_map;\
	source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; \
	roslaunch duckietown_description csv2xacro_node.launch tag_map_csv:=$$tag_map  tile_map_csv:=$$tile_map map_name:=$*'

openhouse-dp6a-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; \
	roslaunch duckietown_demos localization.launch map_name:=$*"

openhouse-dp6a_laptop-%: check-environment
	bash -c "source set_ros_master.sh $*; roslaunch duckietown_demos localization_frontend.launch"

openhouse-dp6b: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos mission_planning.launch"

openhouse-dp6b-laptop-%: check-environment
	bash -c "source set_ros_master.sh $*; rqt --force-discover"

openhouse-dp6: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos localization_navigation.launch"

openhouse-dp6-laptop-%: check-environment
	bash -c "source set_ros_master.sh $*; rqt --force-discover"

indefinite-navigation: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos indefinite_navigation.launch"

openhouse-dp3-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos indefinite_navigation.launch  line_detector_param_file_name:=$*"

openhouse-dp5: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos stop_sign_coordination.launch"

openhouse-dp4: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos traffic_light_coordination.launch"
