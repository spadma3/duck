# Package `auto_localization` {#auto_localization}




The function of this package is to localize all the Duckiebots in a Duckietown in an autolab.

## How to use this package

Note: The package is still under development. The usage might change day-by-day.

### On server computer

First, we need to launch tcp-server on the server computer.

On one terminal, set up the Duckietown environment and set ros master to the server computer itself. then

    roslaunch duckietown_demos tcp_server.launch

Next, on the server computer, launch the node that get messages.

    make auto_localization_server

Congrats! You are done with server sides.

### On watchtowers

Start the local-localization function on local watchtower. They will then launch apriltags2 detection, local-postprocessing, publish to server through tcp communication.

    make auto_localization_watchtower

Congrats! Easy peasy huh?

## System Calibration

The auto-localization relies on the local reference Apriltags. The system will first calculate the pose of an Duckiebot w.r.t the reference tag and later on do transfer the pose from the local frame to the global frame. Therefore, to have a precise localization, we need to know the transformation between local frame and the global frame. Of course one could enter the pose of each tag hand-by-hand, but that will be pretty annoying and inefficient. Here we introduce a tool that could calibrate the whole system, provide the transformation of each tag and save them to and yaml file for future usage.

### Before Calibration

The system calibration require the reference tags to be linked to the origin tag. Thus, we might need to put some "linked" tag to make the graph complete. The linked tags are suggested to be put in the overlap region of watchtowers so that they really achieve their function.

The only constraint of putting the linked tag is that don't use the tags that have been used in the town. Always make sure you don't use the same tag twice.

### On server computer

First, we still need to launch tcp-server on the server computer.

On one terminal, set up the Duckietown environment and set ros master to the server computer itself. then

    roslaunch duckietown_demos tcp_server.launch

Next, on the server computer, launch the node that calculate tag transformations.

    make auto_localization_calibration_server

Server side, done.

### On watchtowers

Start the local-localization function on local watchtower. They will then launch apriltags2 detection, local-postprocessing, publish to server through tcp communication.

    make auto_localization_calibration_watchtower

Done.

### Some more talk about System Calibration

The system calibration require the reference tags to be linked to the origin tag.

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
