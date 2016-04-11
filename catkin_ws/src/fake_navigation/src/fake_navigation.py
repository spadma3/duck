#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('fake_navigation')
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from duckietown_msgs.msg import  WheelsCmdStamped
import std_msgs.msg

from ros import rostopic, rosgraph
import tf
import sys
import math

# calculates the distance between two points
def distBetween (a,b):
	dist = math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
	return dist
	
class speed_publisher (object):
	
	RobotDistService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	trajectory = []
	ntraj = -1
	
	def __init__ (self, robot_name, town):
		self.this_robot = robot_name
		# defining the publishers
		self.pub_wheels_cmd = rospy.Publisher("/" + str(robot_name) + "/wheels_driver_node/wheels_cmd",WheelsCmdStamped,queue_size=1)
		
		# defines the trajectory points that the robot will follow. The trajectory depends on the town loaded
		if town == "small_duckietown":
			self.trajectory =[ (3.61,1.54), (8.43,3.69), (8.39,8.89), (-1.62,10.27), (-5.80,16.21), (-13.03,16.21), (-24.84,16.21), (-30.08, 13.99), (-30.08, 6.73), (-29.31,2.4), (-20.56,1.50), (-15.16, 3.19), (-15.16, 11.09), (-14.19, 13.66), (-8.35,13.66), (-4.98,8.38), (3.27, 7.3), (5.94, 5.97), (3.90,4.32), (-13.02, 4.32), (-24.69, 4.32), (-26.78, 5.94), (-26.78,11.33), (-25.30, 13.62), (-20.41,13.62), (-18.09, 12.43), (-18.09, 6.80), (-16.92, 1.13)]
		else:
			sys.exit("ERROR: Town [" + str(town) + "] not available for fake navigation")
		
		# initialization of the trajectory that the robot will follow. It will start with the closest trajectory point
		# given its launch location
		self.NextTrajPoint, self.ntraj = self.getCloserTrajPoint()
		if self.ntraj == -1:
			sys.exit("ERROR: not possible to get closer point to current location of robot " + str(robot_name))
	
	
	# gets the point of the trajectory that is closer to the current location of the robot
	def getCloserTrajPoint(self):
		
		point = (0.0, 0.0)
		ntraj = -1
		
		try:
			dxy = self.RobotDistService("duckiebot_"+str(self.this_robot), "world")
			x = dxy.pose.position.x
			y = dxy.pose.position.y
			print "Initial pos of the robot is ("+ str(x) + "," + str(y) + ")"
			
			n = 0
			minDist = 1000000.0
			for tpoint in self.trajectory:
				dist = distBetween ((x,y), tpoint)
				if dist < minDist:
					ntraj=n
					point = tpoint
					minDist = dist
				n = n +1
			
				
		except rospy.ServiceException as exc:
			print("Fake_navigation for " + str(self.this_robot) + " robot failed with error: " + str(exc))
		
		print "Closer trajectory point to duckiebot " + str(self.this_robot) +" is "+str(point)+" which is the trajectory point number "+str(ntraj)
		return point, ntraj
	
	
	def getNextOrientation(self):
		#rosservice call /gazebo/get_model_state obj1 obj2
		
		incx = 0.0 
		incy = 0.0
		angle = 0.0
		
		try:
			dxy = self.RobotDistService("duckiebot_"+str(self.this_robot), "world")
			xpos = dxy.pose.position.x
			ypos = dxy.pose.position.y
			(r, p, y) = tf.transformations.euler_from_quaternion([dxy.pose.orientation.x, dxy.pose.orientation.y, dxy.pose.orientation.z, dxy.pose.orientation.w])
			#print "robot " + str(self.this_robot) +" orientation:("+str(r)+","+str(p)+","+str(y)+")"
			
			# if the robot location is very close to the traj point, then we change to the next traj point
			dist = distBetween ((xpos,ypos), self.NextTrajPoint)
			if dist < 0.25:
				self.ntraj = self.ntraj + 1
				# if we reached the end of the trajectory we go back to the first element
				if self.ntraj == len(self.trajectory):
					self.ntraj = 0
				self.NextTrajPoint = self.trajectory[self.ntraj]
				print "\n SELECTED next traj point for duckiebot " + str(self.this_robot) + " is " + str(self.NextTrajPoint) + "\n"
			
			incx = self.NextTrajPoint[0] - xpos
			incy = self.NextTrajPoint[1] - ypos
			trajAng = math.atan2(incy, incx)
			#print "TrajAng:" + str(trajAng)
			
			angle = y - trajAng
			
			# normalize between -PI and PI
			while angle > math.pi:
				angle = angle - 2*math.pi
			while angle < -math.pi:
				angle = angle + 2*math.pi

			#print "Calculated next position increment for robot ["+ str(self.this_robot) + "] is (" + str(incx) + "," + str(incy) + ")"
		except rospy.ServiceException as exc:
			print("Fake_navigation for " + str(self.this_robot) + " robot failed with error: " + str(exc))
	
		#print "Angle to destination: "+str(angle)
		return angle 
	
	# Send the next speed command given the current location and orientation of the robot
	def publishSpeedMsg(self):
		
		baseSpeed = 0.7
		
		# get the next change in speed (normalised to the baseSpeed)
		incVel = self.getNextOrientation() * (2*baseSpeed/math.pi)
		
		# convert that change into a speed command
		twist = WheelsCmdStamped()
		h = std_msgs.msg.Header()	
		h.stamp = rospy.Time.now()
		twist.header = h
		twist.vel_right = baseSpeed - incVel
		twist.vel_left = baseSpeed + incVel
		#print "New Velocities: " + str(twist.vel_left) + "," + str(twist.vel_right)
		# and send it to the wheels
		self.pub_wheels_cmd.publish(twist)

		
	def start(self):
		r = rospy.Rate(2) # 2hz
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
		publisher = speed_publisher(this_robot, "small_duckietown")
		
		publisher.start()