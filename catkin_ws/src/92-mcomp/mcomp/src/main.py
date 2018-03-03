#!/usr/bin/env python

import rospy
import lcm
from lcm_types.mcomp_msgs import event_msg
from duckietown_msgs.msg import BoolStamped


def handler(channel, data):
	stop_pub.publish(True)	


#set up ROS publishing node
rospy.init_node('stop_publisher')

global stop_pub = rospy.Publisher("wheels_driver_node/emergency_stop", BoolStamped, queue_size = 1)



#set up LCM listening
lc = lcm.LCM()
subscription = lc.subscribe("MCOMP_EVENT", handler)


try:
	while True:
		lc.handle()
except KeyboardInterrupt:
	pass
