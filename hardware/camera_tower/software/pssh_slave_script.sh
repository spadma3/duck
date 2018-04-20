#!/bin/bash

MASTER_BASH_NAME='.pssh_master_script.sh'
SLAVE_BASH_NAME='.pssh_slave_script.sh'
CMD_CPT='parallel-scp -i -h $HOME/.pssh_hosts_file '
DUCKIEDES='$HOME/duckietown'

#Name for different function
DUCKIETOWN='dkt'
COPYTO='cpM2S'
COPTFROM='cpS2M'

if [ $# -le 0 ]; then
	echo "System Error: No Args!!!"
	exit
fi

case "$1" in
	$DUCKIETOWN )
		if [ $# -le 1 ]; then
			echo "System Error: No Second Args for Duckietown!!!"
			exit
		fi

		#CMD_DKT="cd duckietown && $2"
		#echo $CMD_DKT

		source duckietown/set_vehicle_name.sh $HOSTNAME
		source duckietown/set_ros_master.sh $2

		case $3 in
			detection )
				rosnode kill -a && roslaunch duckietown camera.launch veh:=$HOSTNAME
				;;
			camera )
				rosnode kill -a && roslaunch duckietown camera.launch veh:=$HOSTNAME
				;;
			* )
				cd duckietown && $3
				;;
		esac
		
		exit
		;;
	* )
		echo "Execute what ever thing you type on slave machines."
		CMD_NOR="$1"
		echo $CMD_NOR
		$CMD_NOR
		exit
esac