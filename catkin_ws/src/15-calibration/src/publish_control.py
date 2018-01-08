#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from math import sin, pi


class calibration:
	def __init__(self):
		
		# Initialize the node with rospy
                rospy.init_node('command', anonymous=True)

                # Create publisher
                publisher=rospy.get_param("~veh")+"/wheels_driver_node/wheels_cmd"
                self.pub_wheels_cmd = rospy.Publisher(publisher,WheelsCmdStamped,queue_size=1)

		self.vFinStraight = rospy.get_param('~vfs')
		self.stepStraight = rospy.get_param("~ss")
		self.durationStraight = rospy.get_param("~ds")
		

	def sendCommand(self, vel_right, vel_left):
		# Put the wheel commands in a message and publish
		msg_wheels_cmd = WheelsCmdStamped()
		msg_wheels_cmd.header.stamp = rospy.Time.now()
		msg_wheels_cmd.vel_right =vel_right
		msg_wheels_cmd.vel_left =vel_left
		self.pub_wheels_cmd.publish(msg_wheels_cmd)

	def StraightCalib(self):
		rospy.loginfo("Straight calibration starts")

		for i in range(0,self.vFinStraight,self.stepStraight):
			self.sendCommand(i/100.0, i/100.0)
			rospy.sleep(self.durationStraight)
		self.sendCommand(0, 0)


	def SinCalib(self):
		rospy.loginfo("Sin calibration starts") 
		k = 0.2
		alpha = 0.4
		omega = 0.02
		phi =  pi
		for i in range(0,1000,1):
			self.sendCommand(k*(1+alpha*sin(omega*i)),k*(1+alpha*sin(omega*i+phi))) 		
			rospy.sleep(0.01)
		
		self.sendCommand(0,0)


if __name__ == '__main__':

	calib=calibration()

	calib.StraightCalib()
	rospy.loginfo("Replace your Duckietown at the beginning of the track, the calibration will restart in 10 seconds")
	rospy.sleep(10)
	calib.SinCalib()
	rospy.loginfo("Calibration finished")


