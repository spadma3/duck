#!/usr/bin/env python
import rospy
import cv2
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import IntersectionPose
import numpy as np


class IntersectionVisualizer(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.robot_name = self.SetupParameter("~robot_name", "daisy")

        # set up path planner, state estimator, ...
        self.intersectionLocalizer = IntersectionLocalizer(self.robot_name)
        self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')

        self.sub_img = rospy.Subscriber("~img",
                                        CompressedImage,
                                        self.ImageCallback,
                                        queue_size=1)
        self.sub_pose = rospy.Subscriber("~pose",
                                        IntersectionPose,
                                        self.PoseCallback,
                                        queue_size=1)
        self.pose = np.zeros(3,float)

        rospy.loginfo("[%s] Initialized." % (self.node_name))


    def ImageCallback(self, msg):

        _, img_gray = self.intersectionLocalizer.ProcessRawImage(msg)
        self.intersectionLocalizer.DrawModel(img_gray, self.pose)
        cv2.imshow('Estimate', img_gray)
        cv2.waitKey(1)


    def PoseCallback(self, msg):
        self.pose[0] = msg.x
        self.pose[1] = msg.y
        self.pose[2] = msg.theta


    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def OnShutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # initialize the node with rospy
    rospy.init_node('intersection_visualizer_node', anonymous=False)

    # create the intersection navigation object
    node = IntersectionVisualizer()

    # setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # spin to prevent stopping
    rospy.spin()
