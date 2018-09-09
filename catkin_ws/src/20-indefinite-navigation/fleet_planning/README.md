# Package Fleet Planning {#fleet_planning}

The fleet planning package provides functionality for the high level control a large number of Duckiebots, the visualization of their locations and status on a map of the Duckietown and an interactive taxi service.
It will provide you with a GUI running on a laptop that can be used to generate transportation requests. The Duckiebots will pick up the customer at their location and bring him/her to their desired final destination.

First, we describe what is needed, which includes:

* Robot hardware
* Number of Duckiebots
* Robot setup steps
* Duckietown hardware

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB17-jwd. DB17-jwdl if coordination at intersections desired.

Requires: Camera calibration completed.

</div>

## Duckietown setup notes {#package-Duckietown-setup}

To begin, you need at least one Duckiebot, and a Duckietown.

The Duckiebot needs to be at least a DB17-jwd or, better, a DB17-jwdl.

The Duckietown needs to have at least one apriltag visible at every intersection to allow localization. Make sure to have a description of your map available as a .csv file. For more infos on this see the localization package, or this [document](https://docs.google.com/document/d/1VE2v2Yn8d4wzA8DnPuA429gYzFeV_zTX8rDFCZCKIE0/edit). Also don't forget to transform the csv files to the xacro format using the corresponding script from the Duckietown_description package. Please note that the demo will NOT work if the Duckietown description is not correct.

If you use this demo, you will have to install the fleet communication dependencies. This might take up to 40 minutes. For more, see below.

## Duckiebot setup notes {#package-Duckiebot-setup}

Duckiebots need to have all the dependencies installed. Use the fleet level communication setup developed by the fleet-communications team and follow their instructions to install the necessary software that enables communication between mutiple ROS master nodes. Now identify the name of wifi interface on by running: `ifconfig`. It is probably named `wlan0` on the Duckiebot. The next step is to run the install script as follows:

    ./install_fleet_messaging <wlan_interface> <ip_address>

Where `<wlan_interface>` is the name of the wlan interface as found with `ifconfig` and `<ip_address>` is a randomly chosen IP address of the form 192.168.x.x/24. The exact IP address is not important for fleet planning with a central node and only becomes relevant if one wanted to run communications without a central node. Make sure that you run this install script with different IP addresses for each Duckiebot that should be used in fleet planning.

As a last step, ensure that all the Duckiebots are connected to the same wifi network.

## Master Laptop setup notes {#package-fleet-planning-master-laptop-setup}

For the demo to run you need one laptop that is in the same wifi network as all the Duckiebots and runs the central planning node. All Duckiebots report their location to that node. With that information it plans which Duckiebot picks up which customer. To receive this messages, the laptop also needs the fleet level communication setup. Perform the same steps as you just did on each Duckiebot on the laptop. (I.e. follow the instructions in the "Duckiebot setup notes" section. Running the installation script takes far less time on the laptop than on the Duckiebot.

## Pre-flight checklist {#package-pre-flight}

Check: Duckiebots have communication dependencies installed (described above).

Check: Duckiebots and laptop are connected to the same network

Check: All Duckiebots can be ssh'ed into from your laptop

Check: You got popcorn and refreshments for the taxi customers

## Running the Fleet Planning Package {#package-run}


### Step 1:
Pick a Duckiebot, log in via ssh.

### Step 2: checkout correct branch
Ensure you are using a branch that includes the fleet planning package.

### Step 3: environment
 Prepare environment:

    source environment.sh
    source set_vehicle_name.sh <robot_name>

### Step 4: rebuild using catkin

    make build-catkin

### Step 5: Run the demo!

Run this on the Duckiebot:

CAUTION: in case of malfunction of other parts of the code, e.g. lane following or intersection control, it is advisable to run the package with the flag `joystick_demo:=true` to enable the possibility of manual joystick intervention.

    roslaunch fleet_planning master.launch messaging_iface:=<wlan_interface> messaging_config:=<config_file>

Where `wlan_interface` is the interface you use to connect to the common network of all Duckiebots (probably `wlan0`) and `config_file` is the file needed to setup the communication. The files are provided as part of the fleet communication package under the following path: `catkin_ws/src/30-localization-and-planning/fleet_messaging/config/config_Duckiebot_*.yaml`. Make sure to pick a different file for each Duckiebot. Make sure to provide an absolute path to the configuration file (i.e. /home/<user>/Duckietown/catkin_ws/...)

Wait until all nodes have successfully been initialized. Then proceed with step 6.

Option: Set `joystick_demo:=true`to switch off to take the autonomous control of the Duckiebot out of the loop. This way you can manually steer the Duckiebot through Duckietown and see how the fleet planning software works, with as little dependency on other packages as possible. Pay attention to the terminal output of your Duckiebot to see which exit to take at an intersection. Give the Duckiebot time to localize at intersections.

### Step 6: Environment laptop
On the laptop, checkout a branch including fleet planning as well, rebuild the catkin folder and in the Duckietown root folder run:

    source environment.sh
    source set_ros_master.sh

Take care: *do not* pass an argument to the command set_ros_master.sh! We want a separate master to run on your laptop since the communication between Duckiebot and laptop is done using the fleet-communication software.

### Step 7: Taxi central:
Run on the taxi central your laptop:

    roslaunch fleet_planning master_laptop.sh messaging_iface:=<wlan_interface> messaging_config:=<config_file>

Where `wlan_interface` is the interface you use to connect to the common network with all Duckiebots (probably `wlan0`) and `config_file` is the file needed to setup the communication. The file is provided as part of the fleet messaging package under the following path: `catkin_ws/src/30-localization-and-planning/fleet_messaging/config/config_laptop.yaml`. Make sure to provide an absolute path to the configuration file (i.e. /home/<user>/Duckietown/catkin_ws/...)

### Step 8: Start the GUI

    rqt --force-discover

If you don't see nothing meaningful, start the fleet planning plugin via the drop-down menu Plugins->Fleet Planning.
Note that the GUI was developed and tested on computers with relatively high resolutions (i.e. > 1920x1080); on smaller screens the map may not be shown entirely. The size of the map tiles can be changed in the code, if necessary, but with smaller tiles the readability of the map may suffer.

### Step 9: Have fun!
Place your Duckiebot at an intersection and it will localize and appear on the map in rqt. The taxi central will automatically assign a mission to the Duckiebot at random to keep it moving and not blocking the streets. If you run the fleet planning in the standard mode, hit R1 on the joystick to go into auto pilot mode. The Duckiebot will now follow the instructions from the taxi central. If the joystick_demo is active, use the joystick to control the Duckiebot and follow the instructions from the Duckiebot's terminal output.

Create a new customer request by clicking on the start node and then on the target node of your journey. Click 'Find Plan'. The taxi central will assign the customer to the closest Duckiebot and recalculate its path once it localizes again. The new path will be displayed on the map. You will see how the customer moves with its taxi once he was picked up.

If a Duckiebot does not localize within a certain time window it will be removed from the map.

### Step 10: More Duckiebots!
Once you get bored with only one Duckiebot on the map, or want to expand your business, add a another Duckiebot to your fleet by repeating steps 1-5. You may add a few more Duckiebots like this.

## Troubleshooting {#package-template-troubleshooting}

The fleet planning package depends on many other packages to work well. You may take the lane following and intersection control packages out of the loop by activating the joystick demo. More details at step 5.
