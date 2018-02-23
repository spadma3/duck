# Duckietown parking README


## Final report:
`DUCKUMENTS_ROOT/docs/atoms_85_fall2017_projects/17_parking/30-final-design-document-parking.md`

## Demo instructions:
A full description of the demo can be found at:
`DUCKUMENTS_ROOT/docs/atoms_20_setup_and_demo/30_demos/17_parking.md`

The demo output is a plot that shows where an arbitrarily placed duckietbot stands within the parking boundaries (given at least one AprilTag in view), The output illustrates that the parking pipeline works well up until control is required. Please see below for an example output of a duckiebot placed forward of the parking entrance and commanded to park in space 2:
<center><img figure-caption="Demo Output" src="demo_output.png" style="width: 600px;"/></center>

The simulation can be used to generate a collision free path in a given space from start pose to end pose where the path is constraint such that a car-like robot can traverse on it. This can be used to generate a path in or out a parking space, for overtaking maneuvers or for generating pathes at intersection. The simulation can be run with the following command: 

`./parking_main.py [!start_index] [!end_index]`

where `[!start_index]` and `[!end_index]` are integers to define the entrance (`0`), a parking space (`1-6`), or the exit (`7`). For example:

`./parking_main.py 0 2`

This should produce the following output:

<center><img figure-caption="path 0 to 2" src="path_0_2.png" style="width: 600px;"/></center>

RRT* is used for a path to space 4, the output should look similar to 

`./parking_main.py 0 4`

<center><img figure-caption="path 0 to 4" src="path_0_4.png" style="width: 600px;"/></center>
