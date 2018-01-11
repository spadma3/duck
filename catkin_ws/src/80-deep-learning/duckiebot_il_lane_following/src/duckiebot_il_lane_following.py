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

GRAPH_BASE_PATH = os.environ['DUCKIETOWN_ROOT'] + '/catkin_ws/src/80-deep-learning/duckiebot_il_lane_following/src/'
IMAGE_DIM = (160, 120)


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

        self.node_name = rospy.get_name()
        # thread lock
        self.thread_lock = threading.Lock()

        # constructor of the classifier
        self.bridge = CvBridge()
        self.active = True
        self.stats = Stats()

        # input channel can be 1 or 3, corresponding to grayscale and rgb image respectively
        self.input_channel = rospy.get_param("~input_channel")

        # subscriber, subscribe to compressed image
        self.image_sub = rospy.Subscriber("~compressed", CompressedImage, self.callback, queue_size=1)
        # publisher, publish to control command /robotname/car_cmd_switch_node/cmd

        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Look for enumerated NCS device(s); quit program if none found.
        self.devices = mvnc.EnumerateDevices()
        if len(self.devices) == 0:
            print('No devices found')
            quit()
        # Get a handle to the first enumerated device and open it
        self.device = mvnc.Device(self.devices[0])
        self.device.OpenDevice()

        if self.input_channel == 1:
            GRAPH_PATH = GRAPH_BASE_PATH + 'one_channel.graph'

        if self.input_channel == 3:
            GRAPH_PATH = GRAPH_BASE_PATH + 'three_channel.graph'

        # Read the graph file into a buffer
        with open(GRAPH_PATH, mode='rb') as f:
            self.blob = f.read()
        # Load the graph buffer into the NCS
        self.graph = self.device.AllocateGraph(self.blob)

    def callback(self, image_msg):

        self.stats.received()
        if not self.active:
            return

            # start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
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
        (rows, cols, chans) = image_cv.shape
        # resize image [Image size is defined during training]
        img = cv2.resize(image_cv, IMAGE_DIM, interpolation=cv2.INTER_NEAREST)
        # cut part of the image
        img = img[40:, :, :]
        # Convert image to gray scale
        if self.input_channel == 1:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # tranform 0-255 to 0-1
        img = cv2.normalize(img.astype('float'), None, 0.0, 1.0, cv2.NORM_MINMAX)
        if self.input_channel == 3:
            img = np.transpose(img, axes=[2, 0, 1])
        # Load the image as a half-precision floating point array
        self.graph.LoadTensor(img.astype(numpy.float16), 'user object')
        # Get the results from NCS
        output, userobj = self.graph.GetResult()
        # Print the results
        #print('\n------- predictions --------')
        #print('input channel: ' + str(self.input_channel))
        # first make the array to float data type
        learning_omega = float(output[0])
        #print(learning_omega)

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
    # initialize the node with rospy
    rospy.init_node('duckiebot_il_lane_following', anonymous=False)
    # create the object
    duckiebot_il_lane_following = imitation_lane_following()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        duckiebot_il_lane_following.graph.DeallocateGraph()
        duckiebot_il_lane_following.device.CloseDevice()
        print("Shutting down!!!")
