#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
# Initialize the node with rospy
rospy.init_node('publisher_node', anonymous=False)
# Create publisher
publisher = rospy.Publisher("~topic_b",String,queue_size=1)
# Publish every 1 second
count = 0
while not rospy.is_shutdown():
    msg = String()
    msg.data = "Hello Duckietown! at %s" % count
    count += 1
    publisher.publish(msg)
    rospy.sleep(1.0)
rospy.spin() #Keeps the script for exiting