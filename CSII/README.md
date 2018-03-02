# Hardware Exercises for CSII from J. Tani

TO ALL TEACHING ASSISTANTS

Hi guys! In the following, I will try to explain how to develop more hardware exercises to every of you, even those who
never used Duckietown. I hope that the students will not need to execute the following steps, we should consider
to set up the working stations such that they do not need to set up anything.

# Setup for TAs and working stations

Please make sure that both on your Duckiebot and your Laptop, you're using this branch

    git checkout CSII-hardware-exercises
    git pull origin CSII-hardware-exercises
    
Then you'll need to compile all non-python content on both the Duckiebot and your Laptop

    cd catkin_ws/
    catkin_make

I highly recommend to use the editor Atom in order to create new hardware exercises. If you have your own editor or you're
simply a hardcore coder who likes terminal editors, go on. If not, refer to this 
[page in the Duckiebook](http://book.duckietown.org/master/duckiebook/atom.html#sec:atom "Atom for Duckietown")

After you're back in the root folder of Duckietown, we're ready to go on!

# Execution of Hardware exercise 1

I already wrote some lines for HWExercise 1. If you would like to execute HWExercise1 ex. 1, type (on your Duckiebot)

    make csii-ex1-1
    
This will execute the line following on your Duckiebot with the controller class controller-1.py you find 
in CSII/Exercises/HWExercise1.

In order to steer your Duckiebot and start the lane following, you can either use the controller or my virtual controller. For the virtual controller, install pygame (on your Laptop)

    pip install pygame
    
Then you're ready to use it! Type (on your Laptop)
    
    make virtual-joystick-<vehicle_name>
    
Where <vehicle_name> is your vehicles name (mine is for example lex). You can steer your Duckiebot by using the arrow keys and
'a' to turn on lane following, 's' to stop lane following. For more information, go to
[this page](https://github.com/duckietown/Software/tree/jukindle-devel-virtualjoy/misc/virtualJoy "Virtual Joystick for Duckiebots")

If you would like to edit the parameters, simply edit those controller files. If you're using Atom as described in the
Duckiebook (linked above) and Atom is open on your Laptop, you can type (on your Duckiebot)

    make csii-edit-ex1-1

Don't forget to have a look at the 
[exercise sheet for HWExercise 1!](https://github.com/idsc-frazzoli/CS2_2018Exercises/blob/jukindle-devel/ProgrammingExercise1/CS2_2018_ProgrammingExercise_1/CS2_2018_ProgrammingExercise_1.pdf "Exercise sheet HWExercise 1")

# Creation of own Hardware exercises

There are only two things you need to do (in special cases even just one thing): You can copy the file controller-1.py into 
HWExercise2 or HWExercise3 directories. When executing "make csii-ex3-2 for example, controller-2.py from HWExercise3 will
be used.

If you need to edit some code behind (for example for delays or different sampling rates as I did in HWExercise1), edit 
the lane_controller_node.py in "catkin_ws/src/10-lane-control/lane_control/scripts". Please read the NOTE inthere.

# FAQ

        
If you have any questions, feel free to contact Julien Kindle: jkindle@ethz.ch
