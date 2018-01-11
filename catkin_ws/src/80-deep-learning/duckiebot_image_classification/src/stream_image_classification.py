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

class Stats():
    def __init__(self):
        self.nresets = 0
        self.reset()

    def reset(self):
        self.nresets += 1
        self.t0 = time.time()
        self.nreceived = 0
        self.nskipped = 0
        self.nprocessed = 0

    def received(self):
        if self.nreceived == 0 and self.nresets == 1:
            rospy.loginfo('stream classification received first image.')
        self.nreceived += 1

    def skipped(self):
        self.nskipped += 1

    def processed(self):
        if self.nprocessed == 0 and self.nresets == 1:
            rospy.loginfo('stream classification processing first image.')

        self.nprocessed += 1

    def info(self):
        delta = time.time() - self.t0

        if self.nreceived:
            skipped_perc = (100.0 * self.nskipped / self.nreceived)
        else:
            skipped_perc = 0

        def fps(x):
            return '%.1f fps' % (x / delta)

        m = ('In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' %
             (delta, self.nreceived, fps(self.nreceived),
              self.nprocessed, fps(self.nprocessed),
              self.nskipped, fps(self.nskipped), skipped_perc))
        return m

class stream_classifier(object):
    def __init__(self):
        
        # thread lock
        self.thread_lock = threading.Lock()
        
        # constructor of the classifier
        self.bridge = CvBridge()
        self.active = True
        self.stats = Stats()
        
        # subscriber, subscribe to compressed image
        self.image_sub = rospy.Subscriber("/tianlu/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1)
        # publisher, publish to control command
        
        
        
        
    def callback(self,image_msg):
        
        self.stats.received()
        if not self.active:
            return 
        
        # start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # returns right away 
        
    def processImage(self, image_msg):    
        
        if not self.thread_lock.acquire(False):
            self.stats.skipped()
            # Return immediately if the thread is locked
            return      
        
        try:
            self.processImage_(image_msg)
        finally:
            # release the thread lock
            self.thread_lock.release()
    
    def processImage_(self, image_msg):
        
        self.stats.processed()
 
        # decode from compressed image with OpenCV
        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return
        
        # import image for classification
        (rows,cols,chans) = image_cv.shape
        # resize image [Image size is defined during training]
        img = cv2.resize( image_cv, IMAGE_DIM)
        # Convert RGB to BGR [skimage reads image in RGB, but Caffe uses BGR]
        img = img[:, :, ::-1]
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
                        
if __name__ == '__main__':
    rospy.init_node('stream_classifier', anonymous=True)
    sc = stream_classifier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        graph.DeallocateGraph()
        device.CloseDevice()
        print("Shutting down!!!")


