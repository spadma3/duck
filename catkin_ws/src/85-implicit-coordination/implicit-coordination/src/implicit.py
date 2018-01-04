#!/usr/bin/env python
import rospy
import time
from random import randrange
from duckietown_msgs.msg import BoolStamped
from multivehicle_tracker.msg import Tracklet, TrackletList


class Implicit(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.iteration = 0
        self.detected_bots = {}

        # Setup publishers
        self.pub_implicit_coordination = rospy.Publisher("~flag_intersection_wait_go_implicit", BoolStamped, queue_size=1)
        # Setup subscriber
        self.sub_at_intersection = rospy.Subscriber("~flag_at_intersection", BoolStamped, self.cbCSMA)
        self.sub_detector = rospy.Subscriber("~vehicle_detection_node", TrackletList, self.cbGetBots)

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

    def cbGetBots(self, tracklet_list):
        for bot in tracklet_list.tracklets:
            print Tracklet.STATUS_TRACKING
            if bot.id not in self.detected_bots:
                self.detected_bots[bot.id] = (0.0, bot.x, 0.0, bot.y)
            if bot.status == Tracklet.STATUS_BORN or bot.status == Tracklet.STATUS_TRACKING:
                print "tracking"
                pos_tupel = self.detected_bots[bot.id] # pos_tupel=(old_x,cur_x,old_y,cur_y)
                self.detected_bots[bot.id] = (pos_tupel[1], bot.x, pos_tupel[3], bot.y)
            else:
                print "lost"
                self.detected_bots.pop(bot.id)
        print self.detected_bots

    def DetectMovement(self):
        detection_threshold = 0.02
        for key in self.detected_bots:
            pos_tupel = self.detected_bots[key]
            diff_x = pos_tupel[0] - pos_tupel[1]
            diff_y = pos_tupel[2] - pos_tupel[3]
            if diff_x**2 + diff_y**2 >= detection_threshold**2:
                return True
        return False

    def cbCSMA(self, args=None):
        flag = BoolStamped()
        SlotTime = 0.2  # in seconds, tunable parameter TODO: experiments
        backoff_time = 0.0  # in seconds
        if self.DetectMovement():
            if self.iteration > 0:
                backoff_time = randrange(0, 2**self.iteration - 1) * SlotTime
            flag.data = False
            self.pub_implicit_coordination.publish(flag)
            time.sleep(backoff_time)
            self.iteration += 1
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
