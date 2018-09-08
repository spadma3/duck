#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState, AprilTagsWithInfos, TurnIDandType, MaintenanceState
from std_msgs.msg import Float32, Int16, Bool, String
from geometry_msgs.msg import Point
import time
import math

class MaintenanceControlNode(object):
    def __init__(self):
        self.node_name = "Maintenance Control Node"

        ## setup Parameters
        self.setupParams()

        ## Internal variables
        self.maintenance_state = "NONE"
        self.active_navigation = False
        self.state = "JOYSTICK_CONTROL"
        self.calibration = False

        ## Subscribers
        self.sub_inters_done_detailed = rospy.Subscriber("~intersection_done_detailed", TurnIDandType, self.cbIntersecDoneDetailed)
        self.go_mt_charging = rospy.Subscriber("~go_mt_charging", Bool, self.cbGoMTCharging)
        self.go_mt_full = rospy.Subscriber("~go_mt_full", Bool, self.cbGoMTFull)
        self.calib_done = rospy.Subscriber("~calibration_done", Bool, self.cbCalibrationDone)
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.set_state = rospy.Subscriber("~set_state", String, self.cbSetState)
        self.rdy_at_exit = rospy.Subscriber("~ready_at_exit",  BoolStamped, self.cbReadyAtExit)

        ## Publishers
        self.pub_maintenance_state = rospy.Publisher("~maintenance_state", MaintenanceState, queue_size=1)
        self.pub_in_charger = rospy.Publisher("~in_charger", BoolStamped, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_id_and_type_out", TurnIDandType, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    ##### Begin callback functions #####

    # Executed as soon as Duckiebot leaves charging rails
    def cbReadyAtExit(self, msg):
        if self.maintenance_state == "CHARGING" and msg.data:
            new_state = "WAY_TO_CALIBRATING" if self.calibration else "WAY_TO_CITY"
            self.active_navigation = True
            self.changeMTState(new_state)


    # For manual debugging
    def cbSetState(self, msg):
        self.changeMTState(msg.data)
        self.active_navigation = self.maintenance_state in ["WAY_TO_CHARGING", "WAY_TO_CALIBRATING", "WAY_TO_CITY", "WAY_TO_MAINTENANCE"]

    # Request to go charging only
    def cbGoMTCharging(self, msg):
        if msg.data:
            self.calibration = False
            self.changeMTState("WAY_TO_MAINTENANCE")

    # Request to do full maintenance (charging and calibration)
    def cbGoMTFull(self, msg):
        if msg.data:
            self.calibration = True
            self.changeMTState("WAY_TO_MAINTENANCE")

    # Called as soon as calibration has finished
    def cbCalibrationDone(self, msg):
        if msg.data:
            self.changeMTState("WAY_TO_CITY")
            self.active_navigation = True

    # Executes when intersection is done - this function is used to determine
    # if a maintenance state should be changed by driving into a specific area
    def cbIntersecDoneDetailed(self, msg):

        tag_id = msg.tag_id
        turn_type = msg.turn_type

        # Check if we just drove through an entrance defined in YAML file
        mt_entered = self.isInDict(tag_id, turn_type, self.maintenance_entrance)
        # Check if we just drove through an exit defined in YAML file
        mt_exited = self.isInDict(tag_id, turn_type, self.maintenance_exit)

        # Same scheme as above
        charging_entered = self.isInDict(tag_id, turn_type, self.stations['entrances'])
        charging_exited = self.isInDict(tag_id, turn_type, self.stations['exits'])

        calib_entered = self.isInDict(tag_id, turn_type,self.calibration_station['entrances'])
        calib_exited = self.isInDict(tag_id, turn_type, self.calibration_station['exits'])

        # summarize gates in two lists
        gate_bools = [mt_entered,           mt_exited,  charging_entered,   calib_entered]
        gate_trans = ["WAY_TO_CHARGING",    "NONE",     "CHARGING",         "CALIBRATING"]

        # Change state if Duckiebot drives through any gate
        for i in range(0, len(gate_bools)):
            if gate_bools[i]: self.changeMTState(gate_trans[i])

        # Notify world that we're in a charger
        if charging_entered:
            entered_msg = BoolStamped()
            entered_msg.data = True
            self.pub_in_charger.publish(entered_msg)

        # Turn on active navigation for WAY_TO_ states
        self.active_navigation = self.maintenance_state in ["WAY_TO_CHARGING", "WAY_TO_CALIBRATING", "WAY_TO_CITY", "WAY_TO_MAINTENANCE"]

    # Adjust turn type if sign has a known ID for our path to charger
    def cbTurnType(self, msg):
        tag_id = msg.tag_id
        turn_type = msg.turn_type

        if self.active_navigation:
            new_turn_type = self.getTurnType(self.charger, tag_id, self.maintenance_state)
            if new_turn_type != -1:
                rospy.loginfo("State: " + str(self.maintenance_state) + ", Charger: " + str(self.charger) + ", tag_ID: "  + str(tag_id) + " - therefore driving " + str(new_turn_type))
                turn_type = new_turn_type
            else:
                if self.maintenance_state != "WAY_TO_MAINTENANCE":
                    rospy.loginfo("Active navigation running, but unknown tag ID " + str(tag_id))

        msg.turn_type = turn_type
        self.pub_turn_type.publish(msg)


    ##### END callback functions #####

    ##### BEGIN internal functions #####

    # Check if tag_id&turn_type are in dictionary
    def isInDict(self, tag_id, turn_type, dictionary):
        inDict = False
        if str(tag_id) in dictionary:
            turns = dictionary[str(tag_id)]
            if isinstance(turns, int): # single turntypes for this tag
                inDict = True if (turn_type == turns) else False
            else: # multiple turntypes for this tag
                inDict = True if (turn_type in turns) else False
        return inDict

    # Function to change MT state
    def changeMTState(self, state):
        # Change internal state and publish information
        self.maintenance_state = state
        maintenance_msg = MaintenanceState()
        maintenance_msg.state = state
        self.pub_maintenance_state.publish(maintenance_msg)

        rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        rospy.loginfo("[Maintenance Control Node] MT State: " + str(state))
        rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

        self.active_navigation = self.maintenance_state in ["WAY_TO_CHARGING", "WAY_TO_CALIBRATING", "WAY_TO_CITY", "WAY_TO_MAINTENANCE"]

    # Returns the turn type for an intersection to get to charger
    def getTurnType(self, chargerID, tagID, maintenance_state):
        if maintenance_state == "WAY_TO_CHARGING":
            path = (self.stations['station' + str(chargerID)])['path_in']
        if maintenance_state == "WAY_TO_CALIBRATING":
            path = (self.stations['station' + str(chargerID)])['path_calib']
        if maintenance_state == "WAY_TO_CITY":
            path = self.path_to_city
        if maintenance_state == "WAY_TO_MAINTENANCE":
            path = self.maintenance_entrance

        turn = -1
        # Get turn type if defined in YAML file
        turn = path[str(tagID)] if str(tagID) in path else turn

        return turn

    ##### END internal functions #####

    ##### BEGIN standard functions #####
    def setupParams(self):
        self.maintenance_entrance = self.setupParam("~maintenance_entrance", 0)
        self.maintenance_exit = self.setupParam("~maintenance_exit", 0)
        self.stations = self.setupParam("~charging_stations", 0)
        self.calibration_station = self.setupParam("~calibration_station", 0)
        self.path_to_city = self.setupParam("~path_to_city", 0)
        if not rospy.has_param("/maintenance_charger"): self.charger = self.setupParam("/maintenance_charger", 1)

    def updateParams(self,event):
        self.maintenance_entrance = rospy.get_param("~maintenance_entrance")
        self.maintenance_exit = rospy.get_param("~maintenance_exit")
        self.stations = rospy.get_param("~charging_stations")
        self.calibration_station = rospy.get_param("~calibration_station")
        self.path_to_city = rospy.get_param("~path_to_city")
        self.charger = rospy.get_param("/maintenance_charger")

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[MaintenanceControlNode] Shutdown.")

    ##### END standard functions #####

if __name__ == '__main__':
    rospy.init_node('maintenance_control_node',anonymous=False)
    maintenance_control_node = MaintenanceControlNode()
    rospy.on_shutdown(maintenance_control_node.onShutdown)
    rospy.spin()
