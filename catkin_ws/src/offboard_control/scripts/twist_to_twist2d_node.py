#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import  Twist2DStamped
from geometry_msgs.msg import Twist

class twist_converter(object):
    def __init__(self):
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~cmd_vel", Twist, self.cbTwist, queue_size=1)
        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)
        
        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def cbTwist(self,twist):       
        car_control_msg = Twist2DStamped()
        car_control_msg.v = twist.linear.x
        car_control_msg.omega = twist.angular.z
        self.pub_car_cmd.publish(car_control_msg)

if __name__ == "__main__":
    rospy.init_node("twist_converter",anonymous=False)
    twist_converter_node = twist_converter()
    rospy.spin()
