#!/usr/bin/env python
from __future__ import print_function
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_utils.jpg import image_cv_from_jpg
import cv2
import rospy
import threading
import time
import numpy as np
# Movidius NCS related packages
import mvnc.mvncapi as mvnc
import numpy
import os
import sys
# Movidius user modifiable input parameters
NCAPPZOO_PATH           = os.path.expanduser( '~/workspace/ncappzoo' )
GRAPH_PATH              = NCAPPZOO_PATH + '/caffe/GoogLeNet/graph' 
LABELS_FILE_PATH        = NCAPPZOO_PATH + '/data/ilsvrc12/synset_words.txt'
IMAGE_MEAN              = [ 104.00698793, 116.66876762, 122.67891434]
IMAGE_STDDEV            = 1
IMAGE_DIM               = ( 224, 224 )

# Look for enumerated NCS device(s); quit program if none found.
devices = mvnc.EnumerateDevices()
if len( devices ) == 0:
    print( 'No devices found' )
    quit()
# Get a handle to the first enumerated device and open it
device = mvnc.Device( devices[0] )
device.OpenDevice()
# Read the graph file into a buffer
with open( GRAPH_PATH, mode='rb' ) as f:
    blob = f.read()
# Load the graph buffer into the NCS
graph = device.AllocateGraph( blob )

class stream_classifier:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/tianlu/camera_node/image/compressed", CompressedImage, self.callback)
    def callback(self,data):
        try:
            cv_image = image_cv_from_jpg(data)
        except ValueError as e:
            print(e)
        (rows,cols,chans) = cv_image.shape
        # resize image [Image size is defined during training]
        img = cv2.resize( cv_image, IMAGE_DIM)
        # Mean subtraction & scaling [A common technique used to center the data]
        img = img.astype( numpy.float32 )
        img = ( img - IMAGE_MEAN ) * IMAGE_STDDEV
        # Load the image as a half-precision floating point array
        graph.LoadTensor( img.astype( numpy.float16 ), 'user object' )
        # Get the results from NCS
        output, userobj = graph.GetResult()
        # Print the results
        print('\n------- predictions --------')
        labels = numpy.loadtxt( LABELS_FILE_PATH, str, delimiter = '\t' )
        order = output.argsort()[::-1][:6]
        for i in range( 0, 4 ):
            print ('prediction ' + str(i) + ' is ' + labels[order[i]]) 
            rospy.loginfo("I can see the images from duckieCamera!!!")
def main(args):
    sc = stream_classifier()
    rospy.init_node('stream_classifier', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        graph.DeallocateGraph()
        device.CloseDevice()
        print("Shutting down!!!")
if __name__ == '__main__':
    main(sys.argv)  
