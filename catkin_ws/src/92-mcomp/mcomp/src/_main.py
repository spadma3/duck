#!/usr/bin/env python

import rospy
import lcm
from lcm_types.mcomp_msgs import event_msg, vgps_location_msg, config
from duckietown_msgs.msg import BoolStamped

max_radius = 0
stop_radius = 0

def handler(channel, data):
	global stop_pub
	print 'I got something'
	msg = BoolStamped()
	msg.header.stamp = rospy.Time.now()
	msg.data = True
	stop_pub.publish( msg )	


def localization_callback(channel, data):
	msg = vgps_location_msg.decode(data)

	if msg.vehicle == "afduck":
		print msg.location
	
def get_config(channel, data):
	msg = config.decode(data)
	print msg

# function to publish messages
# not sure where to put this and where to call it
def send_config(mr, sr, l):
	'''
	mr - max radius
	sr - stop radius
	l - latency
	can add more variables to change if neccessary
	'''
	lc = lcm.LCM()
	msg = config()
	msg.max_radius = mr
	msg.stop_radius = sr
	msg.latency = l
	
	lc.publish("CONFIG", msg.encode())

#set up ROS publishing node
rospy.init_node('stop_publisher')

stop_pub = rospy.Publisher("/noahsduck/joy_mapper_node/joystick_override", BoolStamped, queue_size = 1)



#set up LCM listening
lc = lcm.LCM()
subscription = lc.subscribe("MCOMP_EVENT", handler)
gps_sub = lc.subscribe("VISUAL_GPS", localization_callback)
config_sub = lc.subscribe("CONFIG", get_config)

try:
	while True:
		lc.handle()
except KeyboardInterrupt:
	pass
