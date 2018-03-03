#!/usr/bin/env python

import rospy
import lcm
from lcm_types.mcomp_msgs import event_msg
from duckietown_msgs.msg import BoolStamped


def handler(channel, data):
	global stop_pub
	print 'I got something'
	msg = BoolStamped()
	msg.header.stamp = rospy.Time.now()
	msg.data = True
	stop_pub.publish( msg )	


#set up ROS publishing node
rospy.init_node('stop_publisher')

stop_pub = rospy.Publisher("/noahsduck/joy_mapper_node/joystick_override", BoolStamped, queue_size = 1)



#set up LCM listening
lc = lcm.LCM()
subscription = lc.subscribe("MCOMP_EVENT", handler)


try:
	while True:
		lc.handle()
except KeyboardInterrupt:
	pass
