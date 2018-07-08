#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState, AprilTagsWithInfos, TurnIDandType, MaintenanceState
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point
import time
import math
from duckietown_utils import tcp_communication, robot_name

class ChargingControlNode(object):
    def __init__(self):
        self.node_name = "Charging Control Node"

        # Obtain vehicile name
        self.veh_name = robot_name.get_current_robot_name()

        # Active variable, triggered by Maintenance Control Node
        self.active = False

        ## setup Parameters
        self.setupParams()


        # Class variables
        self.ready2go = False # Whether the bot is fully charged or not
        self.maintenance_state = "NONE"
        self.state = "JOYSTICK_CONTROL"

        ## Subscribers
        self.sub_fsm_state = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_mt_state = rospy.Subscriber("~maintenance_state", MaintenanceState, self.cbMaintenanceState)
        self.sub_stop_line = rospy.Subscriber("~at_stop_line", BoolStamped, self.cbStopLine)
        self.sub_ready2go = rospy.Subscriber("~ready2go", Bool, self.setReady2Go)

        self.sub_inters_done_detailed = rospy.Subscriber("~intersection_done_detailed", TurnIDandType, self.cbIntersecDoneDetailed)

        ## Publishers
        self.ready_at_exit = rospy.Publisher("~ready_at_exit", BoolStamped, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1)
        self.pub_go_charging = rospy.Publisher("~go_charging", Bool, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        # Assume the Duckiebot is full at startup, let him drive to charger after drive_time minutes
        self.drive_timer = rospy.Timer(rospy.Duration.from_sec(60*self.drive_time), self.goToCharger, oneshot=True)


    ##### BEGIN callback functions #####

    # If Duckiebot first in charger and ready2go, leave charger
    def cbStopLine(self, msg):
        if self.state == "CHARGING_FIRST_IN_LINE" and msg.data and self.ready2go:
            ready_at_exit_msg = BoolStamped()
            ready_at_exit_msg.header = msg.header
            ready_at_exit_msg.data = self.ready2go
            self.ready_at_exit.publish(ready_at_exit_msg)

    # Callback maintenance state
    def cbMaintenanceState(self, msg):
        # Start timer which calls Duckiebot back to charger after charge_time mins
        if self.maintenance_state == "CHARGING" and msg.state != self.maintenance_state:
            self.drive_timer = rospy.Timer(rospy.Duration.from_sec(60*self.drive_time), self.goToCharger, oneshot=True)

        self.maintenance_state = msg.state


    # Callback on FSM changes
    def cbFSMState(self, state_msg):
        # if we enter charging area, setup timer for leaving again
        if state_msg.state == "IN_CHARGING_AREA" and self.state != state_msg.state:
            self.ready2go = False
            self.charge_timer = rospy.Timer(rospy.Duration.from_sec(60*self.charge_time), self.setReady2Go, oneshot=True)

        self.state = state_msg.state

    ##### END callback functions #####

    ##### BEGIN internal functions #####

    # Set the status to: ready to leave charging area (battery full)
    def setReady2Go(self, event):
        self.ready2go = True
        rospy.loginfo("[Charing Control Node] Requesting the Duckiebot to leave the charger")

    # Request that Duckiebot should drive to maintenance area for charging
    def goToCharger(self, event):
        go_to_charger = Bool()
        go_to_charger.data = True
        self.pub_go_charging.publish(go_to_charger)
        rospy.loginfo("[Charing Control Node] Requesting the Duckiebot to go charging")

        # Reserving a charging spot
        self.reserveChargerSpot()

    def reserveChargerSpot(self):
        charging_stations = tcp_communication.getVariable("charging_stations")
        if charging_stations is not None and charging_stations != "ERROR":
            # Obtain the one with most free spots
            max_free_spots = 0
            for el in charging_stations:
                if (charging_stations[el])['operational'] and (charging_stations[el])['free_spots'] > max_free_spots:
                    max_free_spots = (charging_stations[el])['free_spots']
                    self.charger = (charging_stations[el])['id']
            if max_free_spots == 0:
                rospy.loginfo("[Charing Control Node] WARNING: NO FREE CHARGING SPOTS AVAILABLE")
                return
            resp = tcp_communication.setVariable("charging_stations/station" + str(self.charger) + "/free_spots", max_free_spots-1)
            if resp:
                rospy.loginfo("[Charing Control Node] Reserved spot in charger " + str(self.charger))
        else:
            rospy.loginfo("[Charing Control Node] ERROR")


    ##### END internal functions #####

    ##### BEGIN standard functions #####

    def setupParams(self):
        self.charge_time = self.setupParam("~charge_time", 1)
        self.drive_time = self.setupParam("~drive_time", 2)


    def updateParams(self,event):
        self.charge_time = rospy.get_param("~charge_time")
        self.drive_time = rospy.get_param("~drive_time")


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[ChargingControlNode] Shutdown.")

    ##### END standard functions #####

if __name__ == '__main__':
    rospy.init_node('charging_control_node',anonymous=False)
    charging_control_node = ChargingControlNode()
    rospy.on_shutdown(charging_control_node.onShutdown)
    rospy.spin()
