# Package `auto_localization` {#auto_localization}

The function of this package is to localize all the Duckiebots in a Duckietown in an autolab.

## How to use this package

Note: The package is still under development. The usage might change day-by-day.

### Map Setup

#### Reference tag setup

There should be at least one reference tag in the field of views of each watchtowers. The reference tags could be traffic sign tags or we could put tags there. The only thing to aware is that don't out a tag in the city twice.

#### Map file setup

You need to create your own map file to make the system work! Create a map file on your server computer.

    laptop $ cd ~/duckietown/catkin_ws/src/30-localization-and-planning/auto_localization/config
    laptop $ cp testcircle_origin(dontoverwrite).yaml ![your map].yaml

The `your map`.yaml is the map file for your robotarium, name it whatever you want. Edit `your map`, modify the id of `origin` to your origin tag id, and add all your watchtowers' hostnames to `watchtowers`.

The default map is testcircle.yaml

### Watchtowers setup

Make sure all watchtowers are connected with power cables and ethernet cables. Checkout the newest branch (at the moment: devel-auto-localization-system-calibration). Do catkin_make on all watchtowers.

    watchtower $ git checkout devel-auto-localization-system-calibration
    watchtower $ catkin_make -C catkin_ws

### System Calibration

The auto-localization relies on the local reference Apriltags. The system will first calculate the pose of an Duckiebot w.r.t the reference tag and later on do transfer the pose from the local frame to the global frame. Therefore, to have a precise localization, we need to know the transformation between local frame and the global frame. Of course one could enter the pose of each tag hand-by-hand, but that will be pretty annoying and inefficient. Here we introduce a tool that could calibrate the whole system, provide the transformation of each tag and save them to and yaml file for future usage.

#### Before Calibration

The system calibration require the reference tags to be linked to the origin tag. Thus, we might need to put some "linked" tag to make the graph complete. The linked tags are suggested to be put in the overlap region of watchtowers so that they really achieve their function.

The only constraint of putting the linked tag is that don't use the tags that have been used in the town. Always make sure you don't use the same tag twice.

#### Calibration

After placing the tags, execute these commands on your server computer to start central server for TCP/IP. ![IP_address] should be the IP address of computer you wanna set it as server. ![your map] is the map file of your map.

    laptop $ make auto_localization_calibration_server IP:=![IP_address]

Open another terminal, start calibration procedure on your server computer side.

    laptop $ make auto_localization_calibration_laptop IP:=![IP_address] map=![your map]

Open another terminal and open ssh connections to all watchtowers through [xpanes](#xpanes) and [tmux](#tmux). Execute the command on watchtowers

    duckiebot $ make auto_localization_calibration_watchtower IP:=![IP_address]

#### Some more explanation about System Calibration

The system calibration require the reference tags to be linked to the origin tag.

### System Localization

First on your server computer, execute following commands. ![IP_address] should be the IP address of computer you wanna set it as server. ![your map] is the map file of your map.

    laptop $ make auto_localization_server IP:=![IP_address]

Open another terminal, start localization procedures on your server computer side

    laptop $ make auto_localization_laptop IP:=![IP_address] map=![your map]

Open another terminal and open ssh connections to all watchtowers through [xpanes](#xpanes) and [tmux](#tmux). Execute the command on watchtowers

    duckiebot $ make auto_localization_watchtower IP:=![IP_address]

The system starts working. On server computer, the `~bot_global_poses` topic publishes Duckiebots' poses, reference tags it take reference to, and the camera that detect the Duckiebot.

The system also records all records under `config` folder with .csv file. You could checkout the performances of each Duckiebot in the file.

## System Architecture

###  Nodes

After apriltag detection nodes, these are the nods that contribute to the localization.

*On the Watchtowers*

| Node         | functions of the node                               |
|--------------|-----------------------------------------------------|
| apriltags2_local_localization_node | Transform the pose of Duckiebots from the camera frame to the frame of tags in the Duckietown   |
| pub2server_easy | Subscribe the results of local localization and set variables of those results in the server through TCPIP |

*On the Server*

| Node         | functions of the node                               |
|--------------|-----------------------------------------------------|
| subfserver_easy | Get the variables from the server, and publish them for the usage at the server. |
| absolute_from_relative_position | Transfer all local poses into global poses for each robots. |

### Topics

Here we listed out the topics for every future developers.

*On the Watchtowers*

| Topics       | Data type         |  Publisher         | Subscriber           | Description |
|--------------|-------------------|--------------------|----------------------| ----------- |
| ~apriltags_out | RemapPoseArray (See duckietown_msgs for more info)   |  apriltags2_local_localization_node | pub2server_easy  | Messages contain poses of Duckiebots w.r.t local tags in Duckietown |

*On the Server*

| Topics       | Data type         |  Publisher         | Subscriber           | Description |
|--------------|-------------------|--------------------|----------------------| ----------- |
| ~local_poses | RemapPoseArray (See duckietown_msgs for more info)   |  subfserver_easy | absolute_from_relative_position | Messages contain poses of Duckiebots w.r.t local tags in Duckietown. Should be transfer to World frame in the next step |
| ~bot_global_poses | GlobalPoseArray (See duckietown_msgs for more info)   |  absolute_from_relative_position | (None) | Messages contain poses of Duckiebots w.r.t global frame in Duckietown. |
