logs:
	@echo "$(sep)Demos"
	@echo
	@echo "These are Makefiles for taking logs"

log-minimal: check-environment
	@read -p "Institution? (UdM, NCTU, ETHZ, TTIC): " institution; \
	. ${DUCKIETOWN_ROOT}/environment.sh; \
	. ${DUCKIETOWN_ROOT}/set_ros_master.sh; \
        . ${DUCKIETOWN_ROOT}/set_vehicle_name.sh; \
	roslaunch duckietown make_log.launch veh:=$(vehicle_name) institution:=$$institution

log-full: check-environment
	@read -p "Institution? (UdM, NCTU, ETHZ, TTIC): " institution; \
	. ${DUCKIETOWN_ROOT}/environment.sh; \
	. ${DUCKIETOWN_ROOT}/set_ros_master.sh; \
        . ${DUCKIETOWN_ROOT}/set_vehicle_name.sh; \
	rosbag record -a -o /media/logs/$(vehicle_name)_$$institution
