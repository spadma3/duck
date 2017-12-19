#!/usr/bin/env python
import rospy
import fleet_messaging.commlibs2 as cl
from std_msgs.msg import String

class Receiver(object):
    """Receives message with a certain filter and publishes to an inbox topic"""
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))
        # Wireless Interface
        self.iface = self.setupParameter("~iface", "wlan0")
        # Instantiating up ZMQ subscriber
        self.sub = cl.duckiemq(interface=self.iface, socktype='sub')
        self.sub.setfilter("Fleet_Planning")
        # Instantiating up ROS
        self.publisher = rospy.Publisher("fleet_planning_inbox",String,queue_size=1)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def receive(self):
        """Recieves ZQM serialzed message and deserializes it"""
        message = self.sub.rcv_string()
        rospy.loginfo("[%s] received: %s" % (self.node_name, message))
        return message

    def publish(self):
        """Publishes received messages"""
        self.publisher.publish(self.receive())


# Initialize the node with rospy
rospy.init_node('receiver_node', anonymous=False)
receiver = Receiver()

while not rospy.is_shutdown():
    print("Publishing")
    receiver.publish()

# Runs continuously until interrupted
rospy.spin() 

