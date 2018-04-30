#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped, AprilTagsWithInfos
from std_msgs.msg import Int16 #Imports msg
import copy
import time
import psutil
import signal
import subprocess, shlex

class AutoCalibrationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None

        #Exit condition for the node
        self.calib_done = False

        #Params for U-Turn
        self.lane_follow_override = False
        self.last_tag = 0
        self.timer_called = False

        #Tags to do a U-turn at
        self.tag_id_inter_1 = 149
        self.tag_id_inter_2 = 65

        #Standing at stopline
        self.stopped = False

        #Node active?
        self.active = False

        #Publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_start = rospy.Publisher("~calibration_start",BoolStamped,queue_size=1)
        self.pub_stop = rospy.Publisher("~calibration_stop",BoolStamped,queue_size=1)
        #self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_car_cmd_in = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.publishControl, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
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
        self.mode = msg.state

    #stops the Duckiebot at an intersection
    def cbStop(self, bool_msg):
        if not self.active:
            return
        else:
            if bool_msg.data:
                self.stopped = True
            if not bool_msg.data:
                self.stopped = False

    #starts the calibration procedure (i.e rosbag) and switches the fsm when command to calibrate is sent
    def cbCalib(self, bool_msg):
        if bool_msg.data:
            command = "rosbag record -O /home/megaduck/media/logs/test.bag /megabot05/tag_detections /megabot05/forward_kinematics_node/velocity"
            command = shlex.split(command)
            #dir_save_bagfile="/media/logs"
            self.rosbag_proc = subprocess.Popen(command)#, stdin=subprocess.PIPE, shell=True)
            calibrate = BoolStamped()
            calibrate.data = True
            self.pub_start.publish(calibrate)
            rospy.Timer(rospy.Duration.from_sec(15), self.finishCalib, oneshot=True)

            rospy.loginfo("[%s] Rosbag recording started" %(self.node_name))
    #atm the calibration stops after 10 seconds --> condition needs to change in the future
    def finishCalib(self,event):
        p = psutil.Process(self.rosbag_proc.pid)
        for sub in p.children(recursive=True):
            sub.send_signal(signal.SIGINT)
        stop = BoolStamped()
        stop.data = True
        self.pub_stop.publish(stop)
        rospy.loginfo("[%s] Rosbag recording stopped" %(self.node_name))

    #Tag detections
    def cbTag(self, tag_msgs):
        if not self.active:
            return
        else:
            #Check if seen tab allows to return to maintenance area
            if(self.calib_done == True):
                self.turnToExit(self,tag_msgs)
            else:
                #Do a U-turn if one of these tags is seen
                for detection in tag_msgs.detections:
                    if((detection.id == self.tag_id_inter_1 or detection.id == self.tag_id_inter_2) and (detection.pose.pose.position.x < 1.5) and (self.last_tag != detection.id)):
                        self.lane_follow_override = True
                        self.last_tag = detection.id
                        rospy.loginfo("[%s] U-Turn started at tag %s" %(self.node_name,detection.id))


    #When calibration is done, return to maintenance area
    def turnToExit(self, tag_msgs):
        for taginfo in tag_msgs.infos:
            rospy.loginfo("[%s] taginfo." %(taginfo))
            if(taginfo.tag_type == taginfo.SIGN):
                availableTurns = []
                #go through possible intersection types
                signType = taginfo.traffic_sign_type
                if (signType == taginfo.LEFT_T_INTERSECT):
                    availableTurns = [1] # these mystical numbers correspond to the array ordering in open_loop_intersection_control_node (very bad)
                elif (signType == taginfo.T_INTERSECTION):
                    availableTurns = [0]
                    #now randomly choose a possible direction
                if(len(availableTurns)>0):
                    self.turn_type = availableTurns[0]
                    #self.pub_turn_type.publish(self.turn_type)
                    #rospy.loginfo("Turn type now: %i" %(self.turn_type))

    #Decide which commands should be sent to wheels in calibration mode
    def publishControl(self, msg):
        if not self.active:
            return
        else:
            car_cmd_msg = msg
            #Command to do a U-turn sent
            if self.lane_follow_override:
                self.stopped = False
                if not self.timer_called:
                    rospy.Timer(rospy.Duration.from_sec(2.2), self.updateTimer, oneshot=True)
                    self.timer_called = True
                car_cmd_msg.v = 0.2
                car_cmd_msg.omega = 4
            #Stopped at stop-line
            elif self.stopped:
                car_cmd_msg.v = 0
                car_cmd_msg.omega = 0
            #Do no U-turn
            else:
                car_cmd_msg.v = msg.v*0.5
                car_cmd_msg.omega = msg.omega*0.5
            #Commands to send to the wheels
            self.pub_car_cmd.publish(car_cmd_msg)

    #U-turn timer
    def updateTimer(self,event):
        #U-turn finishes after 2 seconds
        self.lane_follow_override = False
        self.timer_called = False
        self.stopped = False
        rospy.loginfo("[%s] U-Turn stopped" %(self.node_name))

    def cbSwitch(self, msg):
        self.active=msg.data


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
