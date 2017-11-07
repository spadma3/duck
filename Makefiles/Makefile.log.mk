logs:
	@echo "$(sep)Demos"
	@echo
	@echo "These are Makefiles for taking logs"

log-minimal: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; source log_setup.sh; roslaunch duckietown make_log.launch veh:=$VEHICLE_NAME institution:=$INSTITUTION"
