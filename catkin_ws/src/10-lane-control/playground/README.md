# Package `benchmark' {#benchmark}

This package contains the benchmark code used for the controllers project. It basically takes one or more rosbags on a specific folder and evaluates the run of corresponding duckiebot for d_ref and phi_ref and plots them into a diagram. 

Additionally if the rosbag does not contain any pose information, it takes the pictures and calculates the transformation and line segments itsself. It does also paint the values onto the pictures, so it can be add together to a video.

## Output

![output_diagram](https://lh3.googleusercontent.com/02gzVT_8RnbNrhs302AR2N0lQIhWVMHaY-MMiHAkqv7qy5mgrzDeUNcHGDNXm5j5se6ZAp9csHkcWW-VcSqg=w1070-h1324-rw)

Output diagram should look like this.

![output_results](https://lh5.googleusercontent.com/I3HEBDsPfF_HoVKWRhV5kJFVx0SwlNGAHDjgDXpHLEMVgXRdeM2aKJbemNLG8KStx1Kr_gWU8JMnrkBBE3dI=w1070-h1324)

Output data should look like this.

![output_frame](https://lh4.googleusercontent.com/-J7ZsCBaPgR4NbaKcIQW6e0YKwJ9vuzGJ29Iv6em8C-_NRWIZtm0f5No4PW3DKY5ZvD6dfqWzyA6uyvupjwe=w1070-h1324-rw)

Processed frames look like this.

## How to run

Basically you need to navigate to the folder that contains the benchmark code. In this case:

    $ /catkin_ws/src/10-lane-control/playground/

Then execute following code:

    $ python benchmark path/to/folder_containing_rosbags

You can specify the path to a single rosbag or you can specify the general folder containing several rosbags and it will process all those which haven't processed yet and safe the output into a folder "output" unless otherwise defined.
To get help on the flags type:

    $ python benchmark --help

There are following flags:

     --output 	Name of the output folder (Default: 'output')
     --save_images 	Extract and safe all frames from the rosbag (Default: False)
     --preparer	Define other preparer for image pipeline (Default: prep_120_40)
     --fast		Do not process image frames (Default: True)
