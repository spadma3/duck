# Package `pkg_name` {#pkg_name}
This is the first part of our devel saviors package
FIRST OF ALL: WE NEED AN ADDITIONAL LIBRARY: install via: "pip install scikit-image"

THE NORMAL USAGE OF THE CODE IS AS FOLLOWS: (EVERYTHING TO BE RUN ON THE DUCKIEBOT!!!)

1. "make demo-lane-following"

2. "roslaunch obst_avoid obst_avoid_lane_follow_light.launch veh:=YOUR_ROBOT_NAME_GOES_HERE (default="dori")"

3. press x ON YOUR JOYSTICK then wait for about 10 seconds and ONLY START THE LANE FOLLOWING MODE IF IN THE TERMINAL WHERE YOU STARTED 2. comes a message IN YELLOW: "!!!!!!!!!!!!!!!!!!!!TRAFO WAS COMPUTED SO WE ARE READY TO GO!!!!!!!!!!!!"

IF THIS YELLOW MESSAGE DID NOT APPEAR: repeat number 3



IF YOU WANT TO FURTHER UNDERSTAND OUR CODE AND LAUNCHING OPTIONS THEN YOU CAN READ THE STUFF BELOW!!! 
ESPECIALLY AT THE VERY BOTTOM THER IS INFORMATION PROVIDED CONCERNING THE EASY DEBUGGING AND POTENTIAL ADAPTIONS OF PARAMETERS IN CASE OF UNDESIRED BEHAVIOUR!!!

The other files can be used as follows:

There are currently two nodes implemented which can be used as follows:
1. THERE IS THE OBSTACLE_DETECTION NODE
in this node we perform the general obstacle detection which is later meant to be run online on our duckiebot! It will also additionally automatically launch the continouus anti instagram node and the image transformer node!!!
THE MAIN OUTPUT OF THIS NODE IS THE TOPIC:
'/"YOUR_ROBOT_NAME/obst_detect/posearray'
this pose array contains an array of all of the detected obstacles, where:
- position.x represents the real WORLD x coordinate of the obstacle
- position.y represents the real WORLD y coordinate of the obstacle
- position.z represents the real WORLD RADIUS of the obstacle (is negative if obstacle behind LANE BOUNDARY)
- orientation.x represents the "most" left pixel of the obstacle in bird view
- orientation.y represents the "most" top pixel of the obstacle in bird view
- orientation.z represents the "most" right pixel of the obstacle in bird view
- orientation.w represents the "most" bottom pixel of the obstacle in bird view

THIS SPECIAL MEANING IS VERY IMPORTANT!!!

the node can be launched via:

roslaunch obst_avoid obst_avoid.launch veh:=YOUR_ROBOT_NAME_GOES_HERE (default="dori") show_marker:= (default=false) show_image:= (default=false) use_ai:= (default=true) ai_interval:= (default=10)

if computational load too high -> set ai_interval to a greater value. This value specifies in which interval you compute new color transformation!! (e.g. set to 100!!!) or use the Node No3!!!

example without visualizing anything: roslaunch obst_avoid obst_avoid.launch veh:=arki

example with visualizing the obstacles in the imageframe: roslaunch obst_avoid obst_avoid.launch veh:=arki show_image:=true

NOTE: although this node is not meant for visualization but rather for fast execution you can still visualize the obstacles with those two parameters if you want to and since the obstacle detection algorithm is performing on a cropped version of the image, here if you turn on show_image, this cropped version is displayed. So the visualisation properties in this node are for making development easy but throughout normal operation we recommend to use our SECOND NODE:

2. THERE IS THE OBSTACLE_DETECTION_NODE_VISUAL
in this node we visualize the output of our obstacle detection (=OBSTACLE_DETECTION_NODE). This node is meant to be run on your personal laptop for verifying what is going on during the drive!!!

FOR THIS NODE TO RUN PROPERLY, THE OBSTACLE_DETECTION_NODE MUST BE RUNNING

this node publishes 2 things:
- the obstacles as markers which can be shown in rviz
- the obstacles marked in the image IMPORTANT: unlike in 1. we here show the whole image!!!

the node can be launched via: 

roslaunch obst_avoid obst_avoid_visual.launch veh:=YOUR_ROBOT_NAME_GOES_HERE (default="dori") show_marker:= (default=true) show_image:= (default=true)

example with visualizing everything: roslaunch obst_avoid obst_avoid_visual.launch veh:=arki

3. THERE IS THE OBSTACLE_DETECTION_NODE_ONLY
This node will only launch our obstacle_detection algorithm without the anti_instagram module and will else perform like the obstacle_detection_node -> this one will subscribe to the raw image from the camera!!!

roslaunch obst_avoid obst_avoid_only.launch veh:=YOUR_ROBOT_NAME_GOES_HERE (default="dori") show_marker:= (default=false) show_image:= (default=false) use_ai:= (default=false)

if computational load too high -> set ai_interval to a greater value. This value specifies in which interval you compute new color transformation!! (e.g. set to 100!!!) or use the Node No3!!!

example without visualizing anything: roslaunch obst_avoid obst_avoid_only.launch veh:=arki

example with visualizing the obstacles in the imageframe: roslaunch obst_avoid obst_avoid_only.launch veh:=arki show_image:=true

4. THERE IS THE OBSTACLE_DETECTION_NODE_LANE_FOLLOWING_LIGHT
this node is designed to be used together with lane follwoing demo and provides the following behaviour: in the lane-following demo you can publish a linear transformation by pressing x-button -> this launchfile subscribes to this transformation and publishes corrected image -> is a lightweight version since the user defines when the transformation has to be calculated (in best case before starting the autonomous drive!!!!) and then it is very lightweight and efficient!!!! (BUT IT IS ONLY USING LINEAR TRANSFORMATION,...)

start via:

roslaunch obst_avoid obst_avoid_lane_follow_light.launch veh:=YOUR_ROBOT_NAME_GOES_HERE (default="dori") show_marker:= (default=false) show_image:= (default=false) 

use like: start the make demo-lane-following then start the node via the launch expression (above) then press x on your joystick and hope that it says that color transform was succesfully published else, press x again until telling that it was sucessfully published (you can also double check via laptop by looking at the images of the topic: /ROBOT_NAME/image_transformer_node/corrected_image/ )


SCRIPTS:

We have created a bunch of useful scripts in order to debug offline. 2 of them help to create the images which can then be used to adapt and evaluate our code efficiently. Let us first start with how to create the images.

Assuming that you have got collected a bag including the raw camera images. Then you can use the launch file create_bag.launch which will in return create a new bag which will only contain the corrected image from the anti instagram module. This file is to be used as follows:
roslaunch obst_avoid create_bag.launch veh:=dori path_save:=/home/niggi/Desktop/1.bag path_play:=/home/niggi/Desktop/bags/Record6/dori_slow_and_full_2017-12-11-14-09-28.bag 
where path_play specifies the path to the bag file which should be played
path_save specifies the path to the bag where it should be saved
and veh specifies the name of the vehicle with which the log was taken

after having created this bag containing the corrected images, we have to extract them by using the script dt-bag-image-extract. This can be used by first going into the file .../duckietown/catkin_ws/src/25-devel-saviors/obst_avoid/scripts and typing e.g.

!THIS IS ONE COMMAND!!!
./dt-bag-image-extract /home/niggi/Desktop/1.bag /dori/image_transformer_node/corrected_image /home/niggi/Desktop/1_pics
!END OF COMMAND!
where the first argument is the path to the previously created bagfile, the second is the topic of interes which we want to transform into an image and the third one is the path to a directory which will be created in order to store the pictures in it!!!

Now, that we have all the pictures we can finally tune our code using two jupyter notebook files. The first one color_thresholds let you load some image and shows what would have been detected as yellow, white and orange.
The second script detector_eval lets you evaluate the overall perfomance on your code by loading all of the pictures in one folder, applying the detector on them, drawing all of the bounding boxes and storing the finally created pictures!! execute here the .py script and not the jupyter notebook file -> type ./dt-detector_eval.py (!this needs also a roscore started in another terminal!)

A third script is provided which can be used to decode an .csv file of thehive.ai.

OTHER DEBUGGING/PARAMETERS TO TUNE:

* in general to infer what is going on it is definitely the best to use the scripts provided in our package
* if you feel that the rates are too low and too computationally heavy, either you start Node No1 with a hugh ai_interval (e.g. ai_interval:=100) or you simply start Node No3 but this one will have worse tracking quality since the images are not corrected!!
* THE FOLLOWING PARAMS ARE ALL TO BE CHANGED IN DETECTOR.PY if not stated other
* self.lower_yellow threshold: this threshold is currenty set quite close to red/orange, so if you can be sure that the color transform etc. works you might change them
* the number of pixels from when an object is considered for evaluation is adaptive towards the homography but if you want to change them by a factor, you can do that by increasing self.obst_thres = 35
* if only obstacles in your vicinity are detected you might on the one hand check your extrinsic calibration or if you want to have a fast debug, just change the reference world point in meters via the parameter self.ref_world_point_x = 1.7 or start the Node No1 with show_image:=true since this will show you where the image is cropped!!!
* if you feel that you have smaller elements that should be detected you can also play around with the value self.major_intertia_thres which has impact in the object_filter function. Especially if you want to detect very little duckies you might want to change the criteria to 10 . But we do not recommend to make it lower than 10 because usually lines have an inertia_tensor_eigenval of right below 10,...
* if you might want to run or actually can run your algorithm at more than our 2-3Hz then you have to adapt the rate in the NODE FILE (not the detector.py) (evt. Since we are also trying to run a kind of obstacle tracking to make verything more robust you might want to track the critical obstacles for more frames if you run at a higher frequency,... this adaption can be made in the obst_tracker function (currently set to 4,..)
* if you want to make the tracker a little more restrictive, the two parameters to play with are on the one hand the y_distance and on the other hand the self.minimum_tracking_distance
* in the future: propably a startup procedure will be added,...




<move-here src='#pkg_name-autogenerated'/>
