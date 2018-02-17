## Duckiebot setup notes 

To use the fleet messaging platform, additional wireless adapters are needed that allow mesh networking (e.g. TP-Link TL-WN822N or TL-WN821N).

## Pre-flight checklist 
This pre-flight checklist describes the steps that ensure that the installation will run correctly:

Check: The additional Wifi adapter is installed and works.

    $ sudo apt-get install build-essential linux-headers-generic dkms git
    $ git clone https://github.com/Mange/rtl8192eu-linux-driver.git
    $ sudo dkms add ./rtl8192eu-linux-driver
    $ sudo dkms install rtl8192eu/1.0

Check: Duckiebots have sufficient battery charge.

## Setup
Some packages are needed to enable the communication beween the Duckiebots, namely Protobuf, ZeroMQ and B.A.T.M.A.N.

To install them, ssh into the Duckiebots and source the environment

    $ cd duckietown
    $ source environment.sh

pull the necessary files from devel-distributed-est-master.

Then find the name of the wifi interface you want to use with iwconfig. (eg. wlx7c8bca1120e0).

    $ iwconfig

Next specify a static IP adress and subnet and write it on a piece of paper, be carefull to not use the same IP on two bots. However, the subnet should stay the same on all bots. (eg. 192.168.15.38/24)

Change to dependecy directory

    $ cd ~/duckietown/catkin_ws/src/30-localization-and-planning/fleet_messaging/dependencies
    
and install everything with one handy script!
  
    $ ./install_fleet_messaging <wifi-iface> <ipaddr>
    
Now you need to alter your network config, for this open the interfaces file:

    $ sudo vim /etc/network/interfaces
    
Change all four instances of wlan0 to wlan1.

After a reboot you are ready to make your Duckiebots talk to each other.

## Usage

Every node (bot, laptop etc.) wanting to communicate needs a properly formatted config file:

    - name: "dist-est" 
      description: "distributed estimation images" 
      port: "12345" 
      pub: "pubtopic" 
      sub: "subtopic" 


Then source the environment and launch the communication node with:

    $ source environment.sh
    $ roslaunch fleet_messaging fleet_messaging.launch
