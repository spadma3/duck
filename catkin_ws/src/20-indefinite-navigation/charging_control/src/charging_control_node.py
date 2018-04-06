#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import time
import math

class ChargingControlNode(object):
    def __init__(self):
        self.node_name = "Charging Control Node"



        ## setup Parameters
        self.setupParams()


        self.state = "JOYSTICK_CONTROL"

        ## Subscribers
        self.sub_segs = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)

        ## Publisher
        self.pub_stop_line_reading = rospy.Publisher("~stop_line_reading", StopLineReading, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)



    def cbFSMState(self, state_msg):
        self.state = state_msg.state
        rospy.loginfo("CHARGING CONTROL SAYS NEW STATE LULULULULU")

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
