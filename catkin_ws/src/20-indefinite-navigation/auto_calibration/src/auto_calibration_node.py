#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped
from std_msgs.msg import String, Int16 #Imports msg
import copy

class AutoCalibrationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None

        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_done = rospy.Publisher("~calibration_start",BoolStamped,queue_size=1)
        self.pub_done = rospy.Publisher("~calibration_stop",BoolStamped,queue_size=1)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_car_cmd_in = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.publishControl, queue_size=1)

    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING") and msg.state == "CALIBRATING":
            # Switch into INTERSECTION_CONTROL mode
            self.mode = msg.state
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
        self.mode = msg.state

    def publishControl(self,msg):
        car_cmd_msg = msg
        car_cmd_msg.v = msg.v*0.1
        car_cmd_msg.omega = msg.omega*0.1
        self.pub_car_cmd.publish(car_cmd_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('auto_calibration_node', anonymous=False)

    # Create the NodeName object
    node = AutoCalibrationNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
