#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from math import sin, pi


# Initialize the node with rospy
rospy.init_node('command', anonymous=True)

# Create publisher
publisher=rospy.get_param("~veh")+"/wheels_driver_node/wheels_cmd"
pub_wheels_cmd = rospy.Publisher(publisher,WheelsCmdStamped,queue_size=1)

rospy.loginfo("Calibration starts")


for i in range(0,80,5):
    # Put the wheel commands in a message and publish
    msg_wheels_cmd = WheelsCmdStamped()
    msg_wheels_cmd.header.stamp = rospy.Time.now()
    msg_wheels_cmd.vel_right = i/100.0
    msg_wheels_cmd.vel_left = i/100.0
    pub_wheels_cmd.publish(msg_wheels_cmd)
    rospy.sleep(0.4)

# Put the wheel commands in a message and publish
msg_wheels_cmd = WheelsCmdStamped()
msg_wheels_cmd.header.stamp = rospy.Time.now()
msg_wheels_cmd.vel_right =0.0
msg_wheels_cmd.vel_left =0.0
pub_wheels_cmd.publish(msg_wheels_cmd)

rospy.loginfo("Please replace your Duckietown at the beginning of the track; calibration will restart in 10 seconds")
rospy.sleep(10)

k = 0.2
alpha = 0.4
omega = 0.02
phi =  pi
for i in range(0,1000,1):
    # Put the wheel commands in a message and publish
    msg_wheels_cmd = WheelsCmdStamped()
    msg_wheels_cmd.header.stamp = rospy.Time.now()
    msg_wheels_cmd.vel_right = k*(1+alpha*sin(omega*i))
    msg_wheels_cmd.vel_left = k*(1+alpha*sin(omega*i+phi))
    pub_wheels_cmd.publish(msg_wheels_cmd)

    rospy.sleep(0.01)

# Put the wheel commands in a message and publish
msg_wheels_cmd = WheelsCmdStamped()
msg_wheels_cmd.header.stamp = rospy.Time.now()
msg_wheels_cmd.vel_right =0.0
msg_wheels_cmd.vel_left =0.0
pub_wheels_cmd.publish(msg_wheels_cmd)


rospy.loginfo("Calibration finished")

