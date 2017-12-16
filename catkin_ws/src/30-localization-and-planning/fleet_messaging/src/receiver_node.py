# !/usr/bin/env python
import rospy
from fleet_messaging import commlibs2
# Imports message type
from std_msgs.msg import String


class Receiver(object):
    def __index__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))

        # Wireless Interface
        self.iface = self.setupParameter(self, "iface", "wlan0")

        self.sub = commlibs2.duckiemq(interface=self.iface, socktype='sub')
        self.sub.setfilter("Hello")

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


# Initialize the node with rospy
rospy.init_node('subscriber_node', anonymous=False)

receiver = Receiver()

def recieve(sub, name):
    while True:
        print(name + ": " + sub.rcv_string())
    return

try:
    thread.start_new_thread(recieve, (receiver.sub, "sub", ))
    #thread.start_new_thread(recieve, (sub2, "sub2", ))
except:
    print("Error on spawning threads")

while True:
    pass

# Runs continuously until interrupted
rospy.spin() 

