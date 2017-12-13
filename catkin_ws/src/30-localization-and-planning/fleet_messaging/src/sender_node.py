#!/usr/bin/env python
import rospy
import time
from include.fleet_messaging import commlibs2
# Initialize the node with rospy
rospy.init_node('sender_node', anonymous=False)
pub = commlibs2.duckiemq(interface="wlx18d6c7094a56", socktype='pub')
# Publish every 1 second
while not rospy.is_shutdown():
    ts = time.time()
    pub.send_string("Hello World at: " + str(ts))
    rospy.sleep(1.0)
rospy.spin() #Keeps the script for exiting