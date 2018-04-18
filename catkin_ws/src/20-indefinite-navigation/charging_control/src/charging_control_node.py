#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState, AprilTagsWithInfos, TurnIDandType, MaintenanceState
from std_msgs.msg import Float32, Int16
from geometry_msgs.msg import Point
import time
import math

class ChargingControlNode(object):
    def __init__(self):
        self.node_name = "Charging Control Node"


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
        self.sub_tags = rospy.Subscriber("~april_tags", AprilTagsWithInfos, self.cbAprilTag)
        #self.go_first = rospy.Subscriber("~go_first", BoolStamped, self.cbGoFirst)
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.inters_done = rospy.Subscriber("~intersection_done", BoolStamped, self.cbIntersecDone)

        ## Publisher
        self.ready_at_exit = rospy.Publisher("~ready_at_exit", BoolStamped, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)



    def cbMaintenanceState(self, msg):
        self.maintenance_state = msg.state
        self.active = True if self.maintenance_state == "CHARGING" else False
        rospy.loginfo("ARE WEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE ACTIVE???????   " + str(self.active))

    # Adjust turn type if sign has a known ID for our path to charger
    def cbTurnType(self, msg):
        self.tag_id = msg.tag_id
        self.turn_type = msg.turn_type

        if self.active:
            new_turn_type = self.getTurnType(self.charger, self.tag_id)
            if new_turn_type != -1:
                self.turn_type = new_turn_type

        self.pub_turn_type.publish(self.turn_type)


    def setReady2Go(self, event):
        self.ready2go = True

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
            #TODO: CHANGE FSM STATE here

            # Leaving charger
            if turn == (station['path_out'])[-1]:
                rospy.loginfo("Leaving charging station")


    def cbFSMState(self, state_msg):
        self.state = state_msg.state

        # if we enter charging area, setup timer for leaving again
        if self.state == "IN_CHARGING_AREA":
            self.ready2go = False
            self.charge_timer = rospy.Timer(rospy.Duration.from_sec(60*self.charge_time), self.setReady2Go, oneshot=True)




    #def cbGoFirst(self, msg):
    #    return

    # Executes every time april tag det detects a tag
    def cbAprilTag(self, tag_msg):
        if not self.active:
            return

        tags = tag_msg.detections
        ready_at_exit = False

        # Check if a "first in line" tag is detected
        for tag in tags:

            if tag.id in self.FIL_tags:
                ready_at_exit = True
                break

        # And let any subscriber know that we're first in line and ready2go
        ready_at_exit_msg = BoolStamped()
        ready_at_exit_msg.header = tag_msg.header
        ready_at_exit_msg.data = ready_at_exit and self.ready2go
        self.ready_at_exit.publish(ready_at_exit_msg)

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
        self.charger = self.setupParam("~charger", 3)


    def updateParams(self,event):
        self.maintenance_entrance = rospy.get_param("~maintenance_entrance")
        self.maintenance_exit = rospy.get_param("~maintenance_exit")
        self.stations = rospy.get_param("~charging_stations")
        self.FIL_tags = rospy.get_param("~charger_FIL_tags")
        self.charge_time = rospy.get_param("~charge_time")
        self.charger = rospy.get_param("~charger")




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
