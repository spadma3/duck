#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped, AprilTagsWithInfos
from std_msgs.msg import Int16
import copy
import time

class AutoCalibrationCalculationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None

        #Node active?
        self.active = False

        #Publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_calc_done = rospy.Subscriber("~calibration_calculation_stop",BoolStamped, queue_size=1)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_car_cmd_in = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.publishControl, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)

    #Car entered calibration calculation mode
    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING_CALC") and msg.state == "CALIBRATING_CALC":
            # Switch into CALIBRATING_CALC mode
            self.mode = msg.state
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
            self.calibration()
        self.mode = msg.state

    #Tag detections
    def cbTag(self, tag_msgs):
        if not self.active:
            return
        else:
            return
    #The calibration calculation will come in this function
    def calibration(self):
        rospy.loginfo("[%s] Calculation started." %(self.node_name))
        rospy.Timer(rospy.Duration.from_sec(5), self.finishCalc, oneshot=True)

    #Exit function for calibration calculation
    def finishCalc(self):
        rospy.loginfo("[%s] Calculation finished." %(self.node_name))
        done = BoolStamped()
        done.data = True
        self.pub_calc_done.publish(done)

    #Decide which commands should be sent to wheels in calibration mode
    def publishControl(self, msg):
        if not self.active:
            return
        else:
            car_cmd_msg = msg
            car_cmd_msg.v = 0
            car_cmd_msg.omega = 0
            self.pub_car_cmd.publish(car_cmd_msg)

    def cbSwitch(self, msg):
        self.active=msg.data

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('auto_calibration_node', anonymous=False)

    # Create the NodeName object
    node = AutoCalibrationCalculationNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
