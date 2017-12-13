#!/usr/bin/env python
import rospy

# Imports message type
from std_msgs.msg import String 

# Initialize the node with rospy
rospy.init_node('subscriber_node', anonymous=False)

sub1 = commlibs2.duckiemq(interface="wlx18d6c71b665e", socktype='sub')
sub1.setfilter("Hello")

def recieve(sub, name):
    while True:
        print(name + ": " + sub.rcv_string())
    return

try:
    thread.start_new_thread(recieve, (sub1, "sub1", ))
    thread.start_new_thread(recieve, (sub2, "sub2", ))
except:
    print("Error on spawning threads")

while True:
    pass

# Runs continuously until interrupted
rospy.spin() 

