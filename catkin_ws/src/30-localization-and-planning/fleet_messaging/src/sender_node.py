#!/usr/bin/env python
import rospy
import fleet_messaging.commlibs2 as cl
from std_msgs.msg import String

class Sender(object):
    """Listens to an outox topic and then sends out msg to communication network"""
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))
        # Instantiates ROS subscriber
        self.subscriber = rospy.Subscriber("fleet_planning_outbox", String, self.to_send_cb)
        # Wireless Interface
        self.iface = self.setupParameter("~iface", "wlan0")
        # ZMQ Publisher
        self.pub = cl.duckiemq(interface=self.iface, socktype='pub')

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def to_send_cb(self, msg):
        """A call back that sends out message to communication network"""
        flag = "Fleet_Planning:"
        mail = "%s %s" % (flag, msg.data)
        sender.pub.send_string(mail)
        rospy.loginfo("Sending: " + mail)


rospy.init_node('sender_node', anonymous=False)
sender = Sender()
rospy.spin() #Keeps the script for exiting