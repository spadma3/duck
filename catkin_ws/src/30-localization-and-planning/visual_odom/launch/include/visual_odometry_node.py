#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import time
import visual_odometry

from std_msgs.msg import String, Int32

# Duckiecall Node
# Author: Teddy Ort
# Inputs: ~number_of_quacks/Int32 - The number of quacks that should be in the message
# Outputs: ~duckiecall/String - The output duckiecall message containing a series of quacks

class VisualOdometryNode(object):
    def __init__(self):
        self.node_name = 'visual_odometry_node'

        # Setup the publisher and subscriber
        #NOTE subscribe to img topic



        rospy.loginfo("[%s] has started", self.node_name)

#    def ImageCallback():



if __name__ == '__main__':
    rospy.init_node('visual_odometry_node', anonymous=False)

    rospy.spin()
