#!/bin/bash

MASTER_BASH_NAME='.pssh_master_script.sh'
SLAVE_BASH_NAME='.pssh_slave_script.sh'
CMD_CPT='parallel-scp -i -h $HOME/.pssh_hosts_file '

#Name for different function
DUCKIETOWN='dkt'
COPYTO='cpM2S'
COPTFROM='cpS2M'

function Duckietown {
	echo $1
	CMD_DKT="cd duckietown && $1"
	echo $CMD_DKT
	$CMD_DKT
	exit
}
function cmd2str {
	local STR=""
	echo $@
	
	#eval $STR
}

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

		PASS
		for i in $@; do
			#echo $STR
			STR="$STR $i"
		done

		Duckietown $2
		;;
	* )
		echo "Execute what ever thing you type on slave machines."
		CMD_NOR="$1"
		echo $CMD_NOR
		$CMD_NOR
		exit
esac