#!/usr/bin/env python
import duckietown_utils as du
import numpy as np
import sys
import cv2
import rospy

def apply_filters(I_input, filters):

    if "flip-vertical" in filters:
        I_input = cv2.flip(I_input, 0)
    if "flip-horizontal" in filters:
        I_input = cv2.flip(I_input, 1)
    if "grayscale" in filters:
        I_input = cv2.cvtColor(I_input, cv2.COLOR_BGR2GRAY)
        #Grayscal in BGR format
        I_input = cv2.cvtColor(I_input, cv2.COLOR_GRAY2BGR)
    if "sepia" in filters:
        k_sepia = np.array([[0.272, 0.534, 0.131],
                            [0.349, 0.686, 0.168],
                            [0.393, 0.769, 0.189]])
        I_input = cv2.transform(I_input,k_sepia)

    return I_input

# Imports message type
from std_msgs.msg import String 

# Define callback function
def callback(msg):
    filters = sys.argv[1].split(":")
    img = msg.data
    I_input = du.rgb_from_ros(img)
    I_input = apply_filters(I_input, filters)

    pub_img.publish(du.d8_compressed_image_from_cv_image(I_input, msg))

def listener():
    rospy.Subscriber("talker/topic_a", String, callback)
    rospy.spin()

if __name__ == '__main__':

    # Initialize the node with rospy
    rospy.init_node('subscriber_node', anonymous=False)
    pub_img = rospy.Publisher("/camera_node/image/filter/compress", String, queue_size=0)

    listener()



