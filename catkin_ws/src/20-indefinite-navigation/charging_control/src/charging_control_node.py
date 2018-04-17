#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState, AprilTagsWithInfos, TurnIDandType
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import Point
import time
import math

class ChargingControlNode(object):
    def __init__(self):
        self.node_name = "Charging Control Node"



        ## setup Parameters
        self.setupParams()

        self.chargingTag = False
        self.ready2go = False
        self.inMaintenanceArea = False
        self.turnType = -1

        self.state = "JOYSTICK_CONTROL"

        ## Subscribers
        self.sub_state = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_tags = rospy.Subscriber("~april_tags", AprilTagsWithInfos, self.cbAprilTag)
        self.go_first = rospy.Subscriber("~go_first", BoolStamped, self.cbGoFirst)

        self.turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)

        ## Publisher
        self.at_exit = rospy.Publisher("~at_exit", BoolStamped, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)



    def cbTurnType(self, msg):
        self.tag_id = msg.tag_id
        self.turn_type = msg.turn_type


    def cbFSMState(self, state_msg):
        self.state = state_msg.state

        if self.state == "INTERSECTION_CONTROL" and self.tag_id == 144 and self.turn_type == 1:
            self.inMaintenanceArea = True
            rospy.loginfo("IIIIIIIN MAAAAAAAAINTENANCE AAAAAAAAAREA")


    def goFirst(self, msg):
        if msg.data:
            self.ready2go = True


    def cbGoFirst(self, msg):
        return

    def cbAprilTag(self, tag_msg):
        tags = tag_msg.detections
        at_exit = False
        for tag in tags:
            rospy.loginfo("We see tag no " + str(tag.id))

            if tag.id == 149:
                at_exit = True

        if at_exit:
            at_exit_msg = BoolStamped()
            at_exit_msg.header = tag_msg.header
            at_exit_msg.data = True
            self.at_exit.publish(at_exit_msg)

    def setupParams(self):
        self.ex = self.setupParam("~ex", 0.22) # distance from the stop line that we should stop

    def updateParams(self,event):
        self.ex = rospy.get_param("~ex")



    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[ChargingControlNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('charging_control_node',anonymous=False)
    charging_control_node = ChargingControlNode()
    rospy.on_shutdown(charging_control_node.onShutdown)
    rospy.spin()
