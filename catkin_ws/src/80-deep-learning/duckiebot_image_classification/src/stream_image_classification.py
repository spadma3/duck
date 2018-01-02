#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import (AntiInstagramTransform, BoolStamped, Segment,
    SegmentList, Vector2D)
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils.jpg import image_cv_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker

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
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("~image", CompressedImage, callback)
    rospy.spin()
    
    
if __name__ == '__main__':
    listener()