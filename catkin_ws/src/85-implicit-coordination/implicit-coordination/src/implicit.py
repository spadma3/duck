#!/usr/bin/env python
import rospy
import time
from random import randrange
from duckietown_msgs.msg import BoolStamped
from multivehicle_tracker.msg import TrackletList


class Implicit(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.bStopline = False
        self.iteration = 0
        self.Pose = [0.0, 0.0]

        # Setup publishers
        self.pub_implicit_coordination = rospy.Publisher("~flag_intersection_wait_go_implicit", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_at_intersection = rospy.Subscriber("~flag_at_intersection", BoolStamped, self.cbStop)
        self.sub_detector = rospy.Subscriber("~vehicle_detection_node", TrackletList, self.cbPose)

        rospy.loginfo("[%s] Initialzed." % (self.node_name))
        rospy.Timer(rospy.Duration.from_sec(1.0), self.cbCSMA)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    # callback functions
    def cbStop(self, msg):
        self.bStopline = msg

    def cbPose(self, msg):
        if msg.detection.data:
            self.Pose.append(float(msg.rho.data))
        else:
            self.Pose.append(self.Pose[0])
        self.Pose.pop(0)

    def DetectMovement(self):
        diff = self.Pose[0] - self.Pose[1]
        if abs(diff) >= detection_threshold:
            return True
        else:
            return False

    def cbCSMA(self, args=None):
        print 'fuck the duck'
        flag = BoolStamped()
        SlotTime = 2.0  # in seconds, tunable parameter TODO: experiments
        backoff_time = 0.0  # in seconds
        if self.DetectMovement():
            self.iteration += 1
            backoff_time = randrange(0, 2**self.iteration - 1) * SlotTime
            flag.data = False
            time.sleep(backoff_time)
        else:
            self.iteration = 0
            flag.data = True
        self.pub_implicit_coordination.publish(flag)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('implicit-coordination', anonymous=False)

    # Create the NodeName object
    node = Implicit()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
