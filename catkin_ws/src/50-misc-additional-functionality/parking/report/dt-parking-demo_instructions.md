# Parking demo instructions {#parking-demo-instructions status=beta}

This is the description for the Duckietown parking demo.

First, we describe what is needed, including:

* Robot hardware
* Number of Duckiebots
* Robot setup steps
* Duckietown hardware

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration DB17-wjd or DB17-wjdl

Requires: Camera calibration completed. 

</div>

## Video of expected results {#demo-template-expected}

First, we show a video of the expected behavior. It can be found at TODO!

## Duckietown setup notes {#demo-template-duckietown-setup}

Here, describe the assumptions about the Duckietown, including:

* Layout (tiles types)
* Instrastructure (traffic lights, wifi networks, ...) required
* Weather (lights, ...)
* April tags on parking lot

Do not write instructions here. The instructions should be somewhere in [the part about Duckietowns](#duckietowns). Here, merely point to them.


## Duckiebot setup notes {#demo-template-duckiebot-setup}

Write here any special setup for the Duckiebot, if needed.
TODO: We don't use any, do we?


Do not write instructions here. The instructions should be somewhere in the appropriate setup part.


## Pre-flight checklist {#demo-template-pre-flight}

The pre-flight checklist describes the steps that are sufficient to
ensure that the demo will be correct:

TODO: @Nils, Brett: describe ROS setup

Check: operation 1 done

Check: operation 2 done

## Demo instructions {#demo-template-run}
### Part A: Simulation
instuctions to reproduce the demo simulation:

Step 1: Switch to the parking branch and go to the simulation folder dt-path-planning.

`git checkout devel-parking`

`cd DUCKIETOWN_ROOT/catkin_ws/src/50-misc-additional-functionality/parking/path_planning/dt-path-planning`

Step 2: Start the parking_main file, input parameters are start position and end position. Numbers between 0 and 7 are accepted. 0: entrance, 1-6: parking space 1-6, 7: exit. Reproduce the Dubins path from entrance (0) to parking space 2 (2). 

`./parking_main.py 0 2`

Compare the terminal output and the opened figure to this one - they should be exactly the same. 



### Part B: run path planning on duckiebot
instructions to reproduce the demo on the duckiebot:

TODO: @Nils, Brett: describe 

Step 1: XXX

Step 2: XXX


## Troubleshooting {#demo-template-troubleshooting}

Add here any troubleshooting / tips and tricks required.

TODO: @Nils, Brett: describe 

## Demo failure demonstration {#demo-template-failure}

Finally, put here a video of how the demo can fail, when the assumptions are not respected.

TODO: @Nils, Brett describe
