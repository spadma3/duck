#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped, AprilTagsWithInfos
from std_msgs.msg import Int16 #Imports msg
import copy
import time
import psutil
import signal
import subprocess, shlex
import os
import pwd

class AutoCalibrationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.car_name = rospy.get_namespace()
        self.user_name = pwd.getpwuid(os.getuid()).pw_name
        self.mode = None

        #Exit condition for the node
        self.calib_done = False

        #Params for U-Turn
        self.lane_follow_override = False
        self.last_tag = 0

        #Standing at stopline
        self.stopped = False

        #Node active?
        self.active = False

        #Publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_start = rospy.Publisher("~calibration_start",BoolStamped,queue_size=1)
        self.pub_stop = rospy.Publisher("~calibration_stop",BoolStamped,queue_size=1)
        self.pub_calc_start = rospy.Publisher("~calibration_calculation_start",BoolStamped, queue_size=1)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_car_cmd_in = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.publishControl, queue_size=1)
        self.sub_at_stop_line = rospy.Subscriber("~at_stop_line", BoolStamped, self.cbStop, queue_size=1)
        self.sub_in_calibration_area = rospy.Subscriber("~in_calib", BoolStamped, self.cbCalib, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)

    #Car entered calibration mode
    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING") and msg.state == "CALIBRATING":
            # Switch into CALIBRATING mode
            self.mode = msg.state
            self.stopped = False
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
            #Node shuts itself down after the calculations are complete
            if self.calib_done:
                done = BoolStamped()
                done.data = True
                self.pub_stop.publish(done)
        self.mode = msg.state

    #stops the Duckiebot at an intersection
    def cbStop(self, bool_msg):
        if not self.active:
            return
        else:
            if bool_msg.data:
                self.stopped = True
                #Should stop at stop line and do its calculation
                self.finishCalib()
            if not bool_msg.data:
                self.stopped = False

    #starts the calibration procedure (in the calculation node) and switches the fsm when command to calibrate is sent
    def cbCalib(self, bool_msg):
        if bool_msg.data:
            calibrate = BoolStamped()
            calibrate.data = True
            self.calib_done = False
            self.pub_start.publish(calibrate)
            rospy.loginfo("[%s] Data recording started" %(self.node_name))

    #stopping the data recording and starting calibration calculations
    def finishCalib(self):
        self.calib_done = True
        done = BoolStamped()
        done.data = True
        rospy.loginfo("[%s] Recording stopped" %(self.node_name))
        self.pub_calc_start.publish(done)

    #Decide which commands should be sent to wheels in calibration mode
    def publishControl(self, msg):
        if not self.active:
            return
        else:
            car_cmd_msg = msg
            if self.stopped:
                car_cmd_msg.v = 0
                car_cmd_msg.omega = 0
            else:
                car_cmd_msg.v = msg.v
                car_cmd_msg.omega = msg.omega
            #Commands to send to the wheels
            self.pub_car_cmd.publish(car_cmd_msg)

    #On-off switch for calibration node
    def cbSwitch(self, msg):
        self.active=msg.data

    ########################################################################################
    #Legacy code
    ########################################################################################

    # #Flag for complete calculation
    # def CalibrationDone(self,msg):
    #     self.calib_done=msg.data


    # #U-turn timer
    # def updateTimer(self,event):
    #     #U-turn finishes after 2 seconds
    #     self.lane_follow_override = False
    #     self.timer_called = False
    #     self.stopped = False
    #     rospy.loginfo("[%s] U-Turn stopped" %(self.node_name))

    # #Tag detections for u-turn
    # def cbTag(self, tag_msgs):
    #     if not self.active:
    #         return
    #     else:
    #         #Do a U-turn if one of these tags is seen
    #         for detection in tag_msgs.detections:
    #             if((detection.id == self.tag_id_inter_1 or detection.id == self.tag_id_inter_2) and (detection.pose.pose.position.x < 1.5) and (self.last_tag != detection.id)):
    #                 self.lane_follow_override = True
    #                 self.last_tag = detection.id
    #                 rospy.loginfo("[%s] U-Turn started at tag %s" %(self.node_name,detection.id))

    ########################################################################################
    #Legacy code done
    ########################################################################################

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
