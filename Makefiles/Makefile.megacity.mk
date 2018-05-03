megacity: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos megacity.launch"

tcp-server: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos tcp_server.launch"

virjoy-%: check-environment
	bash -c "source environment.sh; source set_ros_master.sh $*; source set_vehicle_name.sh $*; python misc/virtualJoy/virtualJoy.py"

parallel-autonomy: check-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch"
