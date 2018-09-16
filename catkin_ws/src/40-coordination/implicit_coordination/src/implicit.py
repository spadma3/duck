#!/usr/bin/env python
import rospy
import time
import rospkg
import os
import yaml
from random import randrange, randint, seed
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, FSMState
from multivehicle_tracker.msg import Tracklet, TrackletList
from std_msgs.msg import Int16


class Implicit(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.active = False
        self.iteration = 0
        self.detected_bots = {}

        self.config = self.setupParameter("~config", "baseline")
        self.cali_file_name = self.setupParameter("~cali_file_name", "default")
        rospack = rospkg.RosPack()
        self.cali_file = rospack.get_path('duckietown') + \
            "/config/" + self.config + \
            "/implicit_coordination/implicit_coordination_node/" +  \
            self.cali_file_name + ".yaml"
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\n"
                          % (self.node_name, self.cali_file))
        self.loadConfig(self.cali_file)

	#set seed
	seed()

        # Setup publishers
        self.pub_coord_cmd = rospy.Publisher('simple_coordinator_node/car_cmd',
                                             Twist2DStamped, queue_size=1)
        self.pub_implicit_coordination = rospy.Publisher(
            "~flag_go_wait", BoolStamped, queue_size=1) # TODO fix me

        # Setup subscriber
        # self.sub_at_intersection = rospy.Subscriber("~flag_at_intersection",
        #                                            BoolStamped, self.cbCSMA)
        self.sub_detector = rospy.Subscriber("~vehicle_detection_node",
                                             TrackletList, self.cbGetBots)
        self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cbFSM)  # TODO fix me

        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.publish_car_cmd)
        rospy.loginfo("[%s] Initialzed." % (self.node_name))
        # rospy.Timer(rospy.Duration.from_sec(1.0), self.cbCSMA)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def loadConfig(self, filename):
        stream = file(filename, 'r')
        data = yaml.load(stream)
        stream.close()
        self.detection_threshold = data['detection_threshold']
        self.iteration_threshold = data['iteration_threshold']
        self.right_priority_thresholdy = data['right_priority_thresholdy']
        self.right_priority_thresholdx = data['right_priority_thresholdx']

        self.right_priority = data['right_priority']
        self.SlotTime = data['SlotTime']
        # set Field of Interest
        self.FOI_right = data['FOI_right']
        self.FOI_front = data['FOI_front']
        self.FOI_left = data['FOI_left']
        rospy.loginfo('[%s] detection_threshold: %.4f' % (self.node_name,
                      self.detection_threshold))
        rospy.loginfo('[%s] iteration_threshold: %.4f' % (self.node_name,
                      self.iteration_threshold))
        rospy.loginfo('[%s] SlotTime: %.4f' % (self.node_name, self.SlotTime))
        rospy.loginfo('[%s] right priority: %s' % (self.node_name,
                                                     self.right_priority))
        rospy.loginfo('[%s] FOI boarders (l,r,f): %.4f, %.4f, %.4f'
                      % (self.node_name, self.FOI_left, self.FOI_right, self.FOI_front))

    # callback functions
    def publish_car_cmd(self,event):
        self.pub_coord_cmd.publish(Twist2DStamped(v=0,omega=0))

    def cbGetBots(self, tracklet_list):
        for bot in tracklet_list.tracklets:
            # check if bot already in dict
            if bot.id not in self.detected_bots:
                self.detected_bots[bot.id] = (0.0, bot.x, 0.0, bot.y)
            if bot.status == Tracklet.STATUS_BORN or \
                    bot.status == Tracklet.STATUS_TRACKING:
                print "tracking"
                # pos_tupel=(old_x,cur_x,old_y,cur_y)
                pos_tupel = self.detected_bots[bot.id]
                self.detected_bots[bot.id] = (pos_tupel[1], bot.x,
                                              pos_tupel[3], bot.y)
            else:
                print "lost"
                self.detected_bots.pop(bot.id)
            print self.detected_bots

    def DetectPotCollision(self):
        for key in self.detected_bots:
            pos_tupel = self.detected_bots[key]
            diff_x = pos_tupel[0] - pos_tupel[1]
            diff_y = pos_tupel[2] - pos_tupel[3]
            #check if in Field of Interest
            if (pos_tupel[1] < self.FOI_front) and (pos_tupel[3] > self.FOI_right) and \
                    (pos_tupel[3] < self.FOI_left):
                if (diff_x**2 + diff_y**2 >= self.detection_threshold**2):
                    return True
                if self.right_priority and \
                        self.right_priority_thresholdy > pos_tupel[3] and \
                        self.right_priority_thresholdx > pos_tupel[1]:
                    return True
                return False
            return False

    def cbFSM(self, msg):
        self.mode = msg.state
        if self.mode == "INTERSECTION_COORDINATION":
            self.active = True
            self.CSMA()

    def CSMA(self):
	print "entering csma"
        while self.active:
            rospy.loginfo("[%s] activated" % (self.node_name))
            flag = BoolStamped()
            backoff_time = 0.0  # in seconds
            time.sleep(0.5)
            if self.DetectPotCollision():
                rospy.loginfo("[%s] potential collision" % (self.node_name))
                if self.iteration > 0:
                    backoff_time = randrange(0, 5)
                flag.data = False
                self.pub_implicit_coordination.publish(flag)
                time.sleep(backoff_time)
                self.iteration += 1
                if self.iteration > self.iteration_threshold:
                    flag.data = True
		    self.pub_implicit_coordination.publish(flag)
		    self.active = False
            if self.detected_bots == None or not self.DetectPotCollision():
                rospy.loginfo("[%s] no potential" % (self.node_name))
                self.iteration = 0
                flag.data = True
            	self.pub_implicit_coordination.publish(flag)
		self.active = False

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('implicit_coordination_node', anonymous=False)

    # Create the NodeName object
    node = Implicit()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
