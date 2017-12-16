#!/usr/bin/env python
import rospy
import time
import fleet_messaging.commlibs2 as cl
# Initialize the node with rospy


class Sender(object):
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))

        # Wireless Interface
        self.iface = self.setupParameter("~iface", "wlan0")
        # ZQM Publisher
        self.pub = cl.duckiemq(interface=self.iface, socktype='pub')

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


rospy.init_node('sender_node', anonymous=False)

# Publish every 1 second
while not rospy.is_shutdown():
    sender = Sender()
    ts = time.time()
    sender.pub.send_string("Hello World at: " + str(ts))
    rospy.sleep(1.0)

rospy.spin() #Keeps the script for exiting