#!/usr/bin/env python
import rospy
import time
from random import randrange
from duckietown_msgs.msg import BoolStamped
from geometry_msgs.msg import PoseStamped


class Implicit(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.bStopline = False
        self.Pose = []
        self.iteration = 0
        rospy.loginfo("[%s] Initialzing." % (self.node_name))

        # Setup publishers
        self.pub_implicit_coordination = rospy.Publisher("~flag_intersection_wait_go_implicit", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_at_intersection = rospy.Subscriber("~flag_at_intersection", BoolStamped, self.cbStop)
        self.sub_detector = rospy.Subscriber("~vehicle_detection_node", PoseStamped, self.cbPose)

        rospy.loginfo("[%s] Initialzed." % (self.node_name))

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    # callback functions
    def cbStop(self, msg):
        self.bStopline = msg

    def cbPose(self, msg):
        self.addPose(msg)

    def addPose(self, pose):
        if len(self.Pose) >= 2:
            self.Pose.pop(0)
            self.Pose.append(pose)

    def DetectMovement(self):
        if not self.Pose(1) - self.Pose(2) == 0:
            return True
        else:
            return False

    def CSMA(self):
        SlotTime = 2.0  # in seconds, tunable parameter TODO: experiments
        backoff_time = 0.0  # in seconds
        if self.DetectMovement:
            self.iteration += 1
            backoff_time = randrange(0, 2**self.iteration - 1) * SlotTime
            time.sleep(backoff_time)
        else:
            self.iteration = 0
            self.pub_implicit_coordination

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('implicit-coordination', anonymous=False)

    # Create the NodeName object
    node = Implicit()

    node.CSMA()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
