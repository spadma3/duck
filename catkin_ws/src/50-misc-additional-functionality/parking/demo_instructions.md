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

First, we show a video of the expected behavior. It can be found
[here](https://youtu.be/dS-TWh8cGXk)


## Duckietown setup notes {#demo-template-duckietown-setup}

Here, describe the assumptions about the Duckietown, including:

* Layout (tiles types)
* Instrastructure (traffic lights, wifi networks, ...) required
* Weather (lights, ...)
* It is assumed that the parking lot has six parking spaces as depicted in the figure below.
  * 4 april tags, which should be placed according to the figure below. From the (0,0) point at the bottom left of the figure below (assuming x is the horizontal axis while y is the vertical axis): place tag #126 at  (10cm,0), tag #128 at (20cm,0), tag #129 at (30cm,0) and tag #131 at (40cm,0).

    ​			
    Do not write instructions here. The instructions should be somewhere in [the part about Duckietowns](#duckietowns). Here, merely point to them.

![Duckietown parking lot](https://raw.githubusercontent.com/duckietown/Software/devel-parking/catkin_ws/src/50-misc-additional-functionality/parking/report/map_0_1.png "Duckietown parking lot")


## Duckiebot setup notes {#demo-template-duckiebot-setup}

Write here any special setup for the Duckiebot, if needed.
No special setup needed

Do not write instructions here. The instructions should be somewhere in the appropriate setup part.


## Pre-flight checklist {#demo-template-pre-flight}

The pre-flight checklist describes the steps that are sufficient to
ensure that the demo will be correct:

Check: Ensure that your bot is in the correct configuration (DB17-wjd or DB17-wjdl)

Check: You have a duck safely secured to your duckiebot

## Demo instructions {#demo-template-run}
### Part A: simulation
instuctions to reproduce the demo simulation:

Step 1: Switch to the parking branch and go to the simulation folder dt-path-planning.

`git checkout devel-parking-jan15`

`cd DUCKIETOWN_ROOT/catkin_ws/src/50-misc-additional-functionality/parking/path_planning/dt-path-planning`

Step 2: Start the parking_main file, input parameters are start position and end position. Numbers between 0 and 7 are accepted. 0: entrance, 1-6: parking space 1-6, 7: exit. Reproduce the Dubins path from entrance (0) to parking space 2 (2). 

`./parking_main.py 0 2`

The terminal output should tell you that collision free path from 0 to 2 was found on stage 1. Furthermore, the path is displayed in the figure.

Step 3: Compute a path from entracne (0) to parking space 4 (4). Dubins paht should fail since there is an obstacle along the way. RRT* should take care of the path planning, the simulation tries to find a path for ~ 20 seconds. Afterwards (hopefully after finding a path) the path is displayed in green and the terminal confirms that a path was found. Compare the opened figure to this one. Since RRT* is based on random sampling the path can look different. 

`./parking_main.py 0 4`

The terminal output should tell you that a collision free path was found in stage 2, the path should be displayed (and all the path pieces in cyan are deleted after stop looking for a better path) 


### Part B: Duckiebot
Instructions to reproduce the demo on the duckiebot:

Step 1: Switch to the parking branch: 
`git checkout devel-parking-jan15`

Step 2: Source your environment:
`source environment.sh`

Step 3: Place your duckiebot anywhere in the parking lot, given that it can see the parking apriltags from a reasonable distance

Step 4: Launch the following file with the appropriate arguments:
`roslaunch parking parking_localization.launch veh:=myvehicle ` where the argument "veh" is the name of your duckiebot

Step 5: The launch file will localize the duckiebot via the apriltags. An output of "A collision free path was found!" will display when a path is found.

Step 6: Your duckiebot will locate itself with respect to the parking spaces and save a figure of its position and heading in a figure in your duckietown/catkin_ws/src/50-misc-additional-functionality/parking/src/ folder as "path.pdf". You can view this image to see how well the duckiebot localized itself within the lot and planned a path to a given space.

Step 7: The default planning space the duckiebot will plan to is space 2. If you would like to plan to a different space, type the following command: `roslaunch parking parking_localization.launch veh:=myvehicle end_space = endspace `  where end_space is an integer from 1 to 6. 

## Troubleshooting {#demo-template-troubleshooting}

### Part A: simulation 

Step 1-2 should not give an error. 

Step 3 can generate an error in case of unlucky sampling. See Demo failure demonstratin.

### Part B: Duckiebot

If the parking apriltags are not view and arranged in the correct order 

## Demo failure demonstration {#demo-template-failure}

### Part A: simulation
Step 3 can lead to errors if we are unlucky with sampling (or do not sample enough).

`Traceback (most recent call last):
  File "./parking_main.py", line 514, in <module>
    path_planning(start_number, end_number)
  File "./parking_main.py", line 466, in path_planning
    px, py, pyaw = RRT_star_path_planning(start_x, start_y, start_yaw, end_x, end_y, end_yaw, obstacles)
  File "./parking_main.py", line 286, in RRT_star_path_planning
    path = rrt.Planning(animation=rrt_star_animation)
  File "/Users/samueln/duckietown/catkin_ws/src/50-misc-additional-functionality/parking/path_planning/dt-path-planning/rrt_star_car.py", line 70, in Planning
    lastIndex = self.get_best_last_index()
  File "/Users/samueln/duckietown/catkin_ws/src/50-misc-additional-functionality/parking/path_planning/dt-path-planning/rrt_star_car.py", line 159, in get_best_last_index
    mincost = min([self.nodeList[i].cost for i in fgoalinds])
ValueError: min() arg is an empty sequence`

This error says that no path was found. It can be reproduced by setting `maxIter = 10` in `parking_main.py` on line 36. Standard value is 100. If you get the error once, just start the script again. If several times, increase set `maxIter = 200` in `parking_main.py` on line 36.


### Part B: Duckiebot
See a video [here](https://youtu.be/dsp-9IumGTs)

We do not have a working demo (in the sense that the duckiebot actually parks itself) because the apriltag detection takes several seconds to calculate (~2.9 seconds), resulting in a slow state update while following our given path. If you would like to see the duckiebot trying to control to the planned path, please run the following: 

`roslaunch parking master.launch veh:=myvehicle localization:=true apriltags:=true /camera/raw:=true /camera/raw/rect:=true LED:=false lane_following:=true`

Once a path is generated (it will tell you a path has been found via the terminal), press R1 on your controller to enter "parking mode".