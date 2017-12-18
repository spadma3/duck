#!/usr/bin/env python
import rospy

# Imports message type
from std_msgs.msg import String 

# Define callback function
def callback(msg):
    s = "Fleet_planning_inbox received: %s" % (msg.data)
    rospy.loginfo(s)

# Initialize the node with rospy
rospy.init_node('subscriber_node', anonymous=False)

# Create subscriber
subscriber = rospy.Subscriber("fleet_planning_inbox", String, callback)

# Runs continuously until interrupted
rospy.spin() 
