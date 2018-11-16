
demos2017:
	@echo "$(sep) 2017 demos"
	@echo
	@echo "These were the 2017 demos."
	@echo
	@echo TODO: to write
	@echo

coordination2017: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos stop_sign_coordination2017.launch"

trafficLights2017: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos traffic_light_coordination.launch"
