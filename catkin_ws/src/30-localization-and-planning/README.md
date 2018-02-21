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

run the following to see if you have correctly installed batman and the wifi adapter drivers

    $ sudo batctl if
    [interface] active // what you should get
    
Now you need to alter your network config, for this open the interfaces file:

    $ sudo vim /etc/network/interfaces
    
Change all four instances of wlan0 to wlan1. (This is so you can still connect to the internet after)

After a reboot you are ready to make your Duckiebots talk to each other.

## Usage
To use the platform you must write a configuration file.

Every node (bot, laptop etc.) that wants to communicate needs a properly formatted config file as follows:

    - name: "dist-est" 
      description: "distributed estimation images" 
      port: "12345" 
      pub: "pubtopic" 
      sub: "subtopic" 
      
`name`:arbritary name to distinguish the package using this platform<br/><br/>
`description`: describe what the channel is used for, not used by package, but should help others<br/><br/>
`port`: port number used for the channel, make sure the port is not being used by another package<br/><br/>
`pub`: the outbox_topic that your package should publish to<br/><br/>
`sub`: the inbox_topic that your package will subscribe to <br/><br/>

Then source the environment and launch the communication node with:

    $ source environment.sh
    $ roslaunch fleet_messaging fleet_messaging.launch

### Message Type
Now as the config file suggests, all you have to do now is simply publish and subscribe to the topics you have specified and the messages should be delivered without you having to do anything else.

However, the platform only understands ROS messages in the form of [ByteMultiArray](http://docs.ros.org/jade/api/std_msgs/html/msg/ByteMultiArray.html)
  and **NOTHING ELSE**. Please put your data into this form before you publish to the your outbox_topic.

