#!/usr/bin/env python
import rospy
import fleet_messaging.commlibs2 as cl
from std_msgs.msg import String

class Sender(object):
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))

        self.subscriber = rospy.Subscriber("topic", String, self.callback)

        # Wireless Interface
        self.iface = self.setupParameter("~iface", "wlan0")
        # ZQM Publisher
        self.pub = cl.duckiemq(interface=self.iface, socktype='pub')

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def callback(msg):
        s = "I heard: %s" % (msg.data)
        sender.pub.send_string(s + " and Hello World at: " + str(ts))
        rospy.loginfo(s)


rospy.init_node('sender_node', anonymous=False)
sender = Sender()
rospy.spin() #Keeps the script for exiting