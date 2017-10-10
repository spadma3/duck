#!/usr/bin/env python

import rospy
import rospkg
from micromvp_test.msg import micromvp_carspeed

def callback(data):
	print "Enter callback", data.carID, "lspeed: ", data.lspeed

def Subspeed_node():
	rospy.init_node('subspeed', anonymous=True)
	
	rospy.Subscriber('micromvp', micromvp_carspeed, callback, queue_size=20)

	print "Enter subspeed_node"

	rospy.spin()

if __name__ == '__main__': 
	Subspeed_node()