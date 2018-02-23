# Duckietown parking README


## Final report:
`DUCKUMENTS_ROOT/docs/atoms_85_fall2017_projects/17_parking/30-final-design-document-parking.md`

## Demo instructions:
A full description of the demo can be found at:
`DUCKUMENTS_ROOT/docs/atoms_20_setup_and_demo/30_demos/17_parking.md`

The simulation can be used to generate a collision free path in a given space from start pose to end pose where the path is constraint such that a car-like robot can traverse on it. This can be used to generate a path in or out a parking space, for overtaking maneuvers or for generating pathes at intersection. The simulation can be run with the following command: 

`./parking_main.py [!start_index] [!end_index]`

where `[!start_index]` and `[!end_index]` are integers to define the entrance (`0`), a parking space (`1-6`), or the exit (`7`). For example:

`./parking_main.py 0 2`

This should produce the following output:

