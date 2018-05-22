#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped
from std_msgs.msg import Int16
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import copy
import time

class AutoCalibrationCalculationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.triggered = False
        self.count_last = 0

        #Node active?
        self.active = False

        #Publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_calc_done = rospy.Publisher("~calibration_calculation_stop",BoolStamped, queue_size=1)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_tags = rospy.Subscriber("~tag",AprilTagDetectionArray, self.cbTag, queue_size=10)

    #Car entered calibration calculation mode
    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING_CALC") and msg.state == "CALIBRATING_CALC":
            # Switch into CALIBRATING_CALC mode
            self.mode = msg.state
            self.triggered = True
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
            self.calibration()
            self.publishControl()
        self.mode = msg.state

    #The calibration calculation will come in this function
    def calibration(self):
        if not self.active:
            return
        rospy.loginfo("[%s] Calculation started." %(self.node_name))
        if self.triggered:
            rospy.Timer(rospy.Duration.from_sec(5.0), self.finishCalc, oneshot=True)
            self.triggered = False

    #Exit function for calibration calculation
    def finishCalc(self, event):
        rospy.loginfo("[%s] Calculation finished." %(self.node_name))
        done = BoolStamped()
        done.data = True
        self.pub_calc_done.publish(done)

    def cbTag(self, msg):
        count = 0
        for detection in msg.detections:
            count=count+1
        # if count!=0 and self.count_last!=0:
        #     rospy.loginfo("[%s] Calculation started %s - %s" %(self.node_name, count, self.count_last))
        self.count_last = count

    #Decide which commands should be sent to wheels in calibration mode
    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = 0
        car_cmd_msg.omega = 0
        self.pub_car_cmd.publish(car_cmd_msg)

    def cbSwitch(self, msg):
        self.active=msg.data

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('auto_calibration_calculation_node', anonymous=False)

    # Create the NodeName object
    node = AutoCalibrationCalculationNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
