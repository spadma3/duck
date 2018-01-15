#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from math import sin, pi, cos
import os 

class calibration:
	def __init__(self):
		
		# Initialize the node with rospy
                rospy.init_node('test', anonymous=True)

                # Create publisher
                publisher=rospy.get_param("~veh")+"/joy_mapper_node/car_cmd"
                self.pub_wheels_cmd = rospy.Publisher(publisher,Twist2DStamped,queue_size=1)

#                self.omega = rospy.get_param("~omega")
		self.omega=2.0*pi / 5.0

	def sendCommand(self, vel_right, vel_left):
		# Put the wheel commands in a message and publish
		msg_wheels_cmd = Twist2DStamped()
		msg_wheels_cmd.header.stamp = rospy.Time.now()
		msg_wheels_cmd.v =vel_right
		msg_wheels_cmd.omega =vel_left
		self.pub_wheels_cmd.publish(msg_wheels_cmd)

        def straight(self):
                for n in range(0,10):
                        self.sendCommand(0.2, 0)
                        rospy.sleep(0.5)
                self.sendCommand(0, 0)

        def turn(self):
                for i in range(0,10):
                        self.sendCommand(0.2,self.omega)
                        rospy.sleep(0.5)
                self.sendCommand(0,0)


if __name__ == '__main__':
        calib=calibration()
        rospy.loginfo("straight")
        rospy.sleep(5)
        calib.straight()
        rospy.loginfo("prepare for circle")
        rospy.sleep(5)
        rospy.loginfo("circle")
        calib.turn()

        #os.system("rosnode kill /record")
