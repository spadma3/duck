#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import Image
from duckietown_utils.image_conversions import bgr_from_imgmsg, d8n_image_msg_from_cv_image
from duckietown_utils.image_rescaling import d8_image_resize_no_interpolation
 

class ResizeNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.sub = rospy.Subscriber("~image",Image,self.cbImg, queue_size=1)
        self.pub = rospy.Publisher("~image_resize",Image, queue_size=1)
        
    def cbImg(self,msg):

        img=bgr_from_imgmsg(msg)
        img_resized = d8_image_resize_no_interpolation(img,[64,64])
        img_msg = d8n_image_msg_from_cv_image(img_resized,'bgr8')
        self.pub.publish(img_msg)


if __name__ == '__main__': 
    rospy.init_node('resize_node',anonymous=False)
    node = ResizeNode()
    rospy.spin()

