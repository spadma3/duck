#!/usr/bin/env python
import rospy
import fleet_messaging.commlibs2 as cl

from std_msgs.msg import String


class Receiver(object):
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))

        # Wireless Interface
        self.iface = self.setupParameter("~iface", "wlan0")
        self.sub = cl.duckiemq(interface=self.iface, socktype='sub')
        self.sub.setfilter("Hello")
        self.publisher = rospy.Publisher("~topic",String,queue_size=1)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


# Initialize the node with rospy
rospy.init_node('receiver_node', anonymous=False)

receiver = Receiver()

def receive(sub, name):
    while True:
        message = name + ": " + sub.rcv_string()
        rospy.loginfo("[%s] received: %s" % (name, message))
    return message

while not rospy.is_shutdown():
    receiver.publisher.publish(receive(receiver.sub, receiver.node_name))

# Runs continuously until interrupted
rospy.spin() 

