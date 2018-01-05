#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import BoolStamped, Twist2DStamped
from math import pi
from std_msgs.msg import Bool
class LookAsideNode():

    # replace Bool through BoolStamped
    def __init__(self):
        self.node_name=rospy.get_name()

        self.look_aside_pub = rospy.Publisher("~flag_looking_aside",
				Bool,  queue_size = 1)
        self.car_cmd_pub = rospy.Publisher("~car_cmd",
                                           Twist2DStamped, queue_size=1)

#        self.at_intersection_sub = rospy.Subscriber("~flag_at_intersection", BoolStamped, self.cbStop)
        self.implicit_coordination_sub = rospy.Subscriber("~flag_intersection_wait_go_implicit", Bool, self.cbImplicit, queue_size=1)
#        self.pub_car_cmd = rospy.Publisher("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)
        self.v = 0
        self.omega = 0
        
        self.angle=pi/8
        self.duration=0.5
        self.angular_velocity=self.angle/self.duration




    def cbWheelSwitch(self,switch_msg):
        rospy.loginfo('SWITCH OUTPUT : (left = %.2f, right = %.2f)' %
            (switch_msg.vel_left, switch_msg.vel_right))

    def cbImplicit(self, Bool):
        car_cmd_msg_current = Twist2DStamped()
        car_cmd_msg_current.v=0
        start_time = rospy.Time.now()
        middle_time = start_time + rospy.Duration.from_sec(self.duration)
        wait_time=start_time + 2*rospy.Duration.from_sec(self.duration)
        end_time = start_time+3*rospy.Duration.from_sec(self.duration)
        while rospy.Time.now() < middle_time:
            car_cmd_msg_current.omega=self.angular_velocity
            self.car_cmd_pub.publish(car_cmd_msg_current)
        
        while rospy.Time.now() < wait_time:
            car_cmd_msg_current.omega=0
            self.car_cmd_pub.publish(car_cmd_msg_current)
        while rospy.Time.now() < end_time:
            car_cmd_msg_current.omega = -self.angular_velocity
            self.car_cmd_pub.publish(car_cmd_msg_current)
        car_cmd_msg_current.omega = 0
        self.car_cmd_pub.publish(car_cmd_msg_current)





if __name__ == '__main__':
    rospy.init_node('look-aside', anonymous=False)
    look_asider=LookAsideNode()
    rospy.spin()
