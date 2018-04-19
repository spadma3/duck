#!/bin/bash

MASTER_BASH_NAME='.pssh_master_script.sh'
SLAVE_BASH_NAME='.pssh_slave_script.sh'

CMD_NOR="parallel-ssh -i -h $HOME/.pssh_hosts_file "
CMD_CPT="parallel-scp -h $HOME/.pssh_hosts_file "

# IMPORTANT!! Slave should have same username or copy function will not work.
SLAVE_HOME="/home/tlo/"

#Name for different function
DUCKIETOWN='dkt'
COPYTO='cpM2S'
COPTFROM='cpS2M'

#Functions

function Duckietown {
	echo $1
	CMD_NOR="$CMD_NOR./$SLAVE_BASH_NAME dkt $1"
	echo $CMD_NOR
	$CMD_NOR
	exit
}
function Copy_master2slave {
	echo "Copy this to destination: $1"

	SOURCE=$1
	DESTINATION=$SLAVE_HOME$2
	CMD_CPT="$CMD_CPT $SOURCE $DESTINATION"
	echo $CMD_CPT
	$CMD_CPT

	exit
}
function Copy_slave2master {
	echo $1
	exit
}


#Main Code

if [ $# -le 0 ]; then
	echo "Please specify a function you want to do with pssh."
	echo "Type $MASTER_BASH_NAME --help for more info."
	exit
fi

case "$1" in
	$DUCKIETOWN )
		if [ $# -le 1 ]; then
			echo "You need another argument for Duckietown(dkt)."
			echo "Type $MASTER_BASH_NAME --help for more info."
			exit
		fi

		Duckietown $2
		;;

	$COPYTO )
		if [ $# -le 1 ]; then
			echo "You need to specify a file and destination like this."
			echo "$MASTER_BASH_NAME cpM2S (SOURCE) (DESTINATION) "
			exit
		fi

		Copy_master2slave $2 $3
		;;

	$COPTFROM )
		if [ $# -le 1 ]; then
			echo "You need to specify a machine and a file like this."
			echo "$MASTER_BASH_NAME cpM2S (MACHINE) (FILE) "
			exit
		fi
		
		Copy_slave2master $2 $3
		;;

	* )
		echo "Execute whatever thing you type on slave machines."
		CMD_NOR="$CMD_NOR $1"
		echo $CMD_NOR
		$CMD_NOR
		exit
esac
		