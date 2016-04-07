#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('fake_navigation')
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from duckietown_msgs.msg import  WheelsCmdStamped
import std_msgs.msg

from ros import rostopic, rosgraph
import sys
import math


class speed_publisher (object):
	
	RobotDistService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	
	def __init__ (self, robot_name):
		self.this_robot = robot_name
		# defining the publishers
		self.pub_wheels_cmd = rospy.Publisher("/" + str(robot_name) + "/wheels_driver_node/wheels_cmd",WheelsCmdStamped,queue_size=1)
	
	def getNextStep(self):
		#rosservice call /gazebo/get_model_state obj1 obj2
		
		incx = 0.0 
		incy = 0.0
		
		try:
			dxy = self.RobotDistService("duckiebot_"+str(self.this_robot), "world")
			x = dxy.pose.position.x
			y = dxy.pose.position.y
			if x < 4.22 and x > -25.07:
				nextx = x
				if y < 5.04 and y > 3.48:
					incx = 2.0
					nextx = x + incx
					incy = -0.011 * nextx + 4.2 - y
				if y < 2.1 and y > 0.67: 
					incx = -2.0
					nextx = x - incx
					incy =  y - (0.0013 * nextx + 1.39) 
				
			print "Calculated next position increment for robot"+ str(self.this_robot) + " is (" + str(incx) + "," + str(incy) + ")"
		except rospy.ServiceException as exc:
			print("Fake_navigation for " + str(self.this_robot) + " did not process request: " + str(exc))
	
		return incx, incy
	
	# Send the next speed command given the current location and orientation of the robot
	def publishSpeedMsg(self):
		
		baseSpeed = 0.5
		
		# get the next change in position of the robot
		incx, incy = self.getNextStep()
		
		# convert that change into a speed command
		twist = WheelsCmdStamped()
		h = std_msgs.msg.Header()	
		h.stamp = rospy.Time.now()
		twist.header = h
		twist.vel_right = baseSpeed
		print "atan2:" + str(math.atan2(incy, incx))
		twist.vel_left = baseSpeed + math.atan2(incy, incx)/50
		print "New Velocities: " + str(twist.vel_left) + "," + str(twist.vel_right)
		# and send it to the wheels
		self.pub_wheels_cmd.publish(twist)

		
	def start(self):
		r = rospy.Rate(5) # 2hz
		while not rospy.is_shutdown():
			self.publishSpeedMsg()
			r.sleep()

# Main function.
if __name__ == '__main__':

	if len(sys.argv) < 2:
		print("usage: fake_navigation.py <robot_name>")
	else:
		# Get this robot name
		this_robot= sys.argv[1]
		
		rospy.init_node('fake_navigation_node')
		rospy.wait_for_service('/gazebo/get_model_state')
	
		rospy.sleep(1.0)
		publisher = speed_publisher(this_robot)
		
		publisher.start()
		
		