#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from duckietown_utils.image_rescaling import d8_image_resize_no_interpolation


class Storage:
    publisher = None

def cbCImg(msg):

    img = rgb_from_imgmsg(msg)
    img_small = d8_image_resize_no_interpolation(img,[64,64])

    img_small_msg = Image()
    img_small_msg.header.stamp = msg.header.stamp
    img_small_msg.data = img_small
    
    # Publish empty msg for easy hz testing
    Storage.publisher.publish(img_small_msg)

if __name__ == '__main__': 
    rospy.init_node('img_size_rescaling', anonymous=False)
    Storage.publisher = rospy.Publisher("~image_rect_rescaled", Image, queue_size=1)
    sub_img = rospy.Subscriber("~image_rect", Image, cbCImg)
    rospy.spin()

