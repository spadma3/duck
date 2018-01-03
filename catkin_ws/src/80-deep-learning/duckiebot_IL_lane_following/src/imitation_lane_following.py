#!/usr/bin/env python
from __future__ import print_function
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_utils.jpg import image_cv_from_jpg
from duckietown_msgs.msg import Twist2DStamped, LanePose
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
GRAPH_PATH              = NCAPPZOO_PATH + '/caffe/Duckietown/duckie.graph' 
LABELS_FILE_PATH        = NCAPPZOO_PATH + '/data/ilsvrc12/synset_words.txt'
IMAGE_DIM               = ( 160, 120 )

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

class imitation_lane_following(object):
    def __init__(self):
        
        # thread lock
        self.thread_lock = threading.Lock()
        
        # constructor of the classifier
        self.bridge = CvBridge()
        self.active = True
        self.stats = Stats()
        
        # subscriber, subscribe to compressed image
        self.image_sub = rospy.Subscriber("/tianlu/camera_node/image/compressed", CompressedImage, self.callback, queue_size=1)
        # publisher, publish to control command /robotname/car_cmd_switch_node/cmd
        
        self.pub_car_cmd = rospy.Publisher("/tianlu/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1) 
        
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
        img = cv2.resize( image_cv, IMAGE_DIM, interpolation=cv2.INTER_NEAREST)
        # cut part of the image
        img = img[40:,:,:]
        # Convert image to gray scale
        img =  cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # tranform 0-255 to 0-1
        img = cv2.normalize(img.astype('float'),None, 0.0 , 1.0, cv2.NORM_MINMAX)
        # Load the image as a half-precision floating point array
        graph.LoadTensor( img.astype( numpy.float16 ), 'user object' )
        # Get the results from NCS
        output, userobj = graph.GetResult()
        # Print the results
        print('\n------- predictions --------')
        # first make the array to float data type
        learning_omega = float(output[0])
        print (learning_omega) 
        
        # set car cmd through ros message
        
        car_control_msg = Twist2DStamped()
        car_control_msg.header = image_msg.header
        car_control_msg.v = 0.386400014162
        car_control_msg.omega = learning_omega
        
        # publish the control command
        self.publishCmd(car_control_msg)   
    
    
    def publishCmd(self, car_cmd_msg):
        
        self.pub_car_cmd.publish(car_cmd_msg)    
    
                        
if __name__ == '__main__':
    rospy.init_node('imitation_lane_following', anonymous=False)
    il = imitation_lane_following()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        graph.DeallocateGraph()
        device.CloseDevice()
        print("Shutting down!!!")


