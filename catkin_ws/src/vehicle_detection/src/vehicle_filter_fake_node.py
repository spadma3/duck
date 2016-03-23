#!/usr/bin/env python
from duckietown_msgs.msg import VehicleCorners, VehiclePose
import rospy
import cv2
import io
import numpy as np
import threading
import 

class VehicleFilterNode(object):
	def __init__(self):
		self.node_name = "Vehicle Filter"

		self.vehicle_estimate = VehiclePose()
		self.vehicle_estimate.rho = 0.001 # this should definitely be small enough that we should stop
		self.sub_corners = rospy.Subscriber("~corners", VehicleCorners, 
				self.cbCorners, queue_size=1)
		self.pub_pose = rospy.Publisher("~pose", VehiclePose, queue_size=1)
		rospy.loginfo("Initialization of [%s] completed" % (self.node_name))

	def cbCorners(self, vehicle_corners_msg):		
		if vehicle_corners_msgs.detection:
			sub.pub_pose.publish(self.vehicle_estimate)


if __name__ == '__main__': 
	rospy.init_node('vehicle_filter_node', anonymous=False)
	vehicle_filter_node = VehicleFilterNode()
	rospy.spin()
