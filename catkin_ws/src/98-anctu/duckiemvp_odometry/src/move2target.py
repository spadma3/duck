#!/usr/bin/python

import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Int32

rospy.init_node('move2target', anonymous=True)

def navi_callback(msg):
	print "navi"
	
	car_cmd_pub = rospy.Publisher("/mvpcrane/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)

	x = msg.position.x
	z = msg.position.z
	
	if x < -0.04:
		car_cmd(0, -7.0, car_cmd_pub)
	elif x > 0.04:
		car_cmd(0, 7.0, car_cmd_pub)
	elif -0.04 < x < -0.01:
		car_cmd(0, -7.0, car_cmd_pub)
	elif 0.01 < x < 0.04:
		car_cmd(0, 7.0, car_cmd_pub)
	elif z > 0.12:
		car_cmd(0.4, 0, car_cmd_pub)
	elif 0.10 < z < 0.12:
		car_cmd(0.4, 0, car_cmd_pub)
	elif z < 0.09:
		car_cmd(-0.4, 0, car_cmd_pub)

	car_stop(car_cmd_pub)


def navi_test(msg):
	print "navi test"
	car_cmd_pub = rospy.Publisher("/mvpcrane/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)

	car_cmd(0.4, 0, car_cmd_pub)
	car_stop(car_cmd_pub)

def car_cmd(vel, omega, pub):

	cmd = Twist2DStamped()
	cmd.v = vel
	cmd.omega = omega

	pub.publish(cmd)

def car_stop(pub):

	rospy.sleep(0.1)
	cmd = Twist2DStamped()
	cmd.v = 0.0
	cmd.omega = 0.0

	pub.publish(cmd)

if __name__ == '__main__':
	print "Get Start"
	apriltag_sub = rospy.Subscriber("/homohrt/target2bot_apriltags/tar_pose", Pose, navi_callback, queue_size = 1)
	test_sub = rospy.Subscriber("/test", Int32, navi_test, queue_size = 1)

	rospy.sleep(0.1)
	rospy.spin()