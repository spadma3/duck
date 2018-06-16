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

        # Active variable, triggered by FSM
        self.active = False

        ## setup Parameters
        self.setupParams()
        self.maintenance_state = "NONE"
        # Class variables
        self.ready2go = False # Whether the bot is fully charged or not

        self.turn_type = -1 # Turn type for intersections
        self.tag_id = -1 # Tag ID from intersections

        self.state = "JOYSTICK_CONTROL"

        ## Subscribers
        self.sub_state = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_state = rospy.Subscriber("~maintenance_state", MaintenanceState, self.cbMaintenanceState)
        #####self.sub_tags = rospy.Subscriber("~april_tags", AprilTagsWithInfos, self.cbAprilTag)
        #self.go_first = rospy.Subscriber("~go_first", BoolStamped, self.cbGoFirst)
        self.sub_stop_line = rospy.Subscriber("~at_stop_line", BoolStamped, self.cbStopLine)

        self.sub_ready2go = rospy.Subscriber("~ready2go", Bool, self.setReady2Go)

        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.inters_done = rospy.Subscriber("~intersection_done", BoolStamped, self.cbIntersecDone)

        ## Publisher
        self.ready_at_exit = rospy.Publisher("~ready_at_exit", BoolStamped, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1)
        self.pub_in_charger = rospy.Publisher("~in_charger", BoolStamped, queue_size=1)
        self.pub_go_charging = rospy.Publisher("~go_charging", Bool, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        # Assume the Duckiebot is full, let him drive to charger after drive_time minutes
        self.drive_timer = rospy.Timer(rospy.Duration.from_sec(60*self.drive_time), self.goToCharger, oneshot=True)


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
        if self.maintenance_state == "CHARGING" and msg.state == "NONE":
            self.drive_timer = rospy.Timer(rospy.Duration.from_sec(60*self.drive_time), self.goToCharger, oneshot=True)

        self.maintenance_state = msg.state
        self.active = True if self.maintenance_state == "CHARGING" else False

    # Adjust turn type if sign has a known ID for our path to charger
    def cbTurnType(self, msg):
        self.tag_id = msg.tag_id
        self.turn_type = msg.turn_type

        if self.active:
            new_turn_type = self.getTurnType(self.charger, self.tag_id)
            if new_turn_type != -1:
                rospy.loginfo("Leading Bot to charger " + str(self.charger) + " - therefore going " + str(new_turn_type) + "at tag " + str(self.tag_id))
                self.turn_type = new_turn_type

        self.pub_turn_type.publish(self.turn_type)

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

    # Executes when intersection is done
    def cbIntersecDone(self, msg):
        if not self.active:
            return
        turn = [self.tag_id, self.turn_type]

        # get parameters of charging station
        station = self.stations['station' + str(self.charger)]

        # Entering charger
        if turn == (station['path_in'])[-1]:
            rospy.loginfo("Entering charging station")
            in_charger = BoolStamped()
            in_charger.header = msg.header
            in_charger.data = True
            self.pub_in_charger.publish(in_charger)

        # Leaving charger
        if turn == (station['path_out'])[-1]:
            rospy.loginfo("Leaving charging station")

    def speedUp(self, event):
        rospy.set_param("/" + self.veh_name +"/lane_controller_node/v_bar", self.v_charger_inside)

    # Callback on FSM changes
    def cbFSMState(self, state_msg):

        # Leaving charging module
        if self.state == "CHARGING_FIRST_IN_LINE" and state_msg.state == "LANE_FOLLOWING":
            rospy.set_param("/" + self.veh_name +"/lane_controller_node/v_bar", self.speed_old)
            rospy.set_param("/" + self.veh_name +"/lane_controller_node/k_Id", self.charger_k_Id_old)

        # if we enter charging area, setup timer for leaving again
        if state_msg.state == "IN_CHARGING_AREA" and self.state != state_msg.state:
            self.ready2go = False
            self.charge_timer = rospy.Timer(rospy.Duration.from_sec(60*self.charge_time), self.setReady2Go, oneshot=True)

            # Adjust speed in charging area and k_Id
            self.speed_old = rospy.get_param("/" + self.veh_name +"/lane_controller_node/v_bar")
            self.charger_k_Id_old = rospy.get_param("/" + self.veh_name +"/lane_controller_node/k_Id")
            rospy.set_param("/" + self.veh_name +"/lane_controller_node/v_bar", self.v_charger_entrance)
            rospy.set_param("/" + self.veh_name +"/lane_controller_node/k_Id", self.charger_k_Id)

            # Speed up after 10s (Duckiebot is then for sure on charging module)
            self.speedup_timer = rospy.Timer(rospy.Duration.from_sec(self.slow_time), self.speedUp, oneshot=True)

        self.state = state_msg.state

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


    # # Executes every time april tag det detects a tag
    # def cbAprilTag(self, tag_msg):
    #     if not self.active:
    #         return
    #
    #     tags = tag_msg.detections
    #     ready_at_exit = False
    #
    #     # Check if a "first in line" tag is detected
    #     for tag in tags:
    #
    #         if tag.id in self.FIL_tags:
    #             ready_at_exit = True
    #             break
    #
    #     # And let any subscriber know that we're first in line and ready2go
    #     ready_at_exit_msg = BoolStamped()
    #     ready_at_exit_msg.header = tag_msg.header
    #     ready_at_exit_msg.data = ready_at_exit and self.ready2go
    #     self.ready_at_exit.publish(ready_at_exit_msg)

    # Returns the turn type for an intersection to get to charger
    def getTurnType(self, chargerID, tagID):
        station = self.stations['station' + str(chargerID)]
        turn = -1
        for el in station['path_in']:
            if el[0] == tagID:
                turn = el[1]
        for el in station['path_out']:
            if el[0] == tagID:
                turn = el[1]
        return turn


    def setupParams(self):
        self.maintenance_entrance = self.setupParam("~maintenance_entrance", 0)
        self.maintenance_exit = self.setupParam("~maintenance_exit", 0)
        self.stations = self.setupParam("~charging_stations", 0)
        self.FIL_tags = self.setupParam("~charger_FIL_tags", 0)
        self.charge_time = self.setupParam("~charge_time", 1)
        self.drive_time = self.setupParam("~drive_time", 2)
        self.charger = self.setupParam("~charger", 3)
        self.v_charger_entrance = self.setupParam("~v_charger_entrance", 0.15)
        self.v_charger_inside = self.setupParam("~v_charger_inside", 0.35)
        self.charger_k_Id = self.setupParam("~charger_k_Id", 0.1)
        self.slow_time = self.setupParam("~slow_time", 20)

    def updateParams(self,event):
        self.maintenance_entrance = rospy.get_param("~maintenance_entrance")
        self.maintenance_exit = rospy.get_param("~maintenance_exit")
        self.stations = rospy.get_param("~charging_stations")
        self.FIL_tags = rospy.get_param("~charger_FIL_tags")
        self.charge_time = rospy.get_param("~charge_time")
        self.drive_time = rospy.get_param("~drive_time")
        self.charger = rospy.get_param("~charger")
        self.v_charger_entrance = rospy.get_param("~v_charger_entrance")
        self.v_charger_inside = rospy.get_param("~v_charger_inside")
        self.charger_k_Id = rospy.get_param("~charger_k_Id")
        self.slow_time = rospy.get_param("~slow_time")

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
