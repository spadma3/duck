#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

import cv2
import rospy
import threading
import time
import numpy as np

import mvnc.mvncapi as mvnc
import skimage
from skimage import io, transform
import numpy
import os
import sys

# subscribe to the published compressed images from duckiebot on-board camera
def callback(data):
    rospy.loginfo("I can see the images from duckieCamera!!!")
def listener():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/tianlu/camera_node/image/compressed", CompressedImage, callback)
    rospy.spin()
    
    
if __name__ == '__main__':
    listener()
