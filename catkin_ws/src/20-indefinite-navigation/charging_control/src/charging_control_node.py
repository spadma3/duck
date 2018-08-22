#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped, StopLineReading, LanePose, FSMState, AprilTagsWithInfos, TurnIDandType, MaintenanceState, Twist2DStamped
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point
import time
import math
from duckietown_utils import tcp_communication, robot_name
import commands

class ChargingControlNode(object):
    def __init__(self):
        self.node_name = "Charging Control Node"

        # Obtain vehicile name
        self.veh_name = robot_name.get_current_robot_name()

        ## setup Parameters
        self.setupParams()


        # Class variables
        self.ready2go = False # Whether the bot is fully charged or not
        self.notChargingCounter = 0
        self.T_check_charging = 4
        self.time_until_wiggle = 24
        self.i2c_addr = 0x42
        self.state_reg = 1

        self.maintenance_state = "NONE"
        self.state = "JOYSTICK_CONTROL"


        ## Subscribers
        self.sub_fsm_state = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_mt_state = rospy.Subscriber("~maintenance_state", MaintenanceState, self.cbMaintenanceState)
        self.sub_stop_line = rospy.Subscriber("~at_stop_line", BoolStamped, self.cbStopLine)
        self.sub_ready2go = rospy.Subscriber("~ready2go", Bool, self.setReady2Go)

        ## Publishers
        self.ready_at_exit = rospy.Publisher("~ready_at_exit", BoolStamped, queue_size=1)
        self.pub_go_mt_charging = rospy.Publisher("~go_mt_charging", Bool, queue_size=1)
        self.pub_go_mt_full = rospy.Publisher("~go_mt_full", Bool, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # State to int mappings
        self.positionStates = {'UNKNOWN_STATE': [0,2,6], 'STANDING_STILL': [1], 'MOVING': [3], 'STANDING_STILL_AND_CHARGING': [4,5], 'MOVING_AND_CHARGING': [7]}
        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        ## wiggle-operation to obtain good conact if not charging
        self.wiggle_timer = rospy.Timer(rospy.Duration.from_sec(self.T_check_charging), self.checkIfCharging)

        # Assume the Duckiebot is full at startup, let him drive to charger after drive_time minutes
        self.drive_timer = rospy.Timer(rospy.Duration.from_sec(60*self.drive_time), self.goToCharger, oneshot=True)


    ##### BEGIN callback functions #####

    # If Duckiebot first in charger and ready2go, leave charger
    def cbStopLine(self, msg):
        if self.state == "CHARGING_FIRST_IN_LINE" and msg.data and self.ready2go:
            self.pubReady(True)

    # Callback maintenance state
    def cbMaintenanceState(self, msg):

        if self.maintenance_state == "NONE" and msg.state == "WAY_TO_MAINTENANCE":
            # Reserving a charging spot
            self.reserveChargerSpot()

        # Start timer which calls Duckiebot back to charger after charge_time mins
        if self.maintenance_state == "CHARGING" and msg.state != self.maintenance_state:
            self.drive_timer = rospy.Timer(rospy.Duration.from_sec(60*self.drive_time), self.goToCharger, oneshot=True)

        self.maintenance_state = msg.state


    # Callback on FSM changes
    def cbFSMState(self, state_msg):
        # if we enter charging area, setup timer for leaving again
        if state_msg.state == "IN_CHARGING_AREA" and self.state != state_msg.state:
            self.ready2go = False
            self.pubReady(False)
            self.charge_timer = rospy.Timer(rospy.Duration.from_sec(60*self.charge_time), self.setReady2Go, oneshot=True)

        self.state = state_msg.state

    ##### END callback functions #####

    ##### BEGIN internal functions #####

    # If we were supposed to charge but not charging for too long, do a small wiggle to get contact again
    def checkIfCharging(self,event):
        if self.state != "IN_CHARGING_AREA":
            self.notChargingCounter = 0
            return

        state_output = commands.getstatusoutput("i2cget -y 1 " + str(self.i2c_addr) + " " + str(self.state_reg))
        if state_output[0] != 0: return

        positionState = int(state_output[1], 0)

        rospy.loginfo("State " + str(positionState))

        if positionState not in (self.positionStates['STANDING_STILL_AND_CHARGING']+self.positionStates['MOVING_AND_CHARGING']):
            self.notChargingCounter += 1
            if self.notChargingCounter*self.T_check_charging >= self.time_until_wiggle:
                rospy.loginfo("[Charging Control Node]: We were not charging for too long, wiggling now.")
                self.doWiggle()
                self.notChargingCounter = 0
        else:
            self.notChargingCounter = 0


    def doWiggle(self):
        carmsg = Twist2DStamped()
        carmsg.v = -0.3
        carmsg.omega = 0
        self.pub_car_cmd.publish(carmsg)
        rospy.sleep(0.1)
        carmsg.v = 0.2
        carmsg.omega = 0
        self.pub_car_cmd.publish(carmsg)
        rospy.sleep(0.1)
        carmsg.v = 0.0
        carmsg.omega = 0
        self.pub_car_cmd.publish(carmsg)


    def pubReady(self, ready):
        ready_at_exit_msg = BoolStamped()
        ready_at_exit_msg.data = ready
        self.ready_at_exit.publish(ready_at_exit_msg)


    # Set the status to: ready to leave charging area (battery full)
    def setReady2Go(self, event):
        self.ready2go = True

        rospy.loginfo("[Charing Control Node] Requesting the Duckiebot to leave the charger")
        self.releaseChargerSpot()

    # Request that Duckiebot should drive to maintenance area for charging
    def goToCharger(self, event):
        go_to_charger = Bool()
        go_to_charger.data = True
        if self.do_calib:
            self.pub_go_mt_full.publish(go_to_charger)
            rospy.loginfo("[Charing Control Node] Requesting the Duckiebot to go charging and calibrating.")
        else:
            self.pub_go_mt_charging.publish(go_to_charger)
            rospy.loginfo("[Charing Control Node] Requesting the Duckiebot to go charging.")



    def releaseChargerSpot(self):
        charging_stations = tcp_communication.getVariable("charging_stations")
        if charging_stations is not None and charging_stations != "ERROR":
            current_free_spots = (charging_stations["station" + str(self.charger)])['free_spots']

            resp = tcp_communication.setVariable("charging_stations/station" + str(self.charger) + "/free_spots", current_free_spots+1)
            if resp:
                rospy.loginfo("[Charing Control Node] Released spot in charger " + str(self.charger))
        else:
            rospy.loginfo("[Charing Control Node] ERROR in TCP communication!")


    def reserveChargerSpot(self):
        charging_stations = tcp_communication.getVariable("charging_stations")
        if charging_stations is not None and charging_stations != "ERROR":
            # Obtain the one with most free spots
            max_free_spots = 0
            for el in charging_stations:
                if (charging_stations[el])['operational'] and (charging_stations[el])['free_spots'] > max_free_spots:
                    max_free_spots = (charging_stations[el])['free_spots']
                    rospy.set_param("/maintenance_charger", (charging_stations[el])['id'])
                    self.charger = (charging_stations[el])['id']
            if max_free_spots == 0:
                rospy.loginfo("[Charing Control Node] WARNING: NO FREE CHARGING SPOTS AVAILABLE")
                return
            resp = tcp_communication.setVariable("charging_stations/station" + str(self.charger) + "/free_spots", max_free_spots-1)
            if resp:
                rospy.loginfo("[Charing Control Node] Reserved spot in charger " + str(self.charger))
        else:
            rospy.loginfo("[Charing Control Node] ERROR in TCP communication!")


    ##### END internal functions #####

    ##### BEGIN standard functions #####

    def setupParams(self):
        self.charge_time = self.setupParam("~charge_time", 1)
        self.drive_time = self.setupParam("~drive_time", 2)
        self.do_calib = self.setupParam("~do_calib", False)
        if not rospy.has_param("/maintenance_charger"): self.charger = self.setupParam("/maintenance_charger", 1)

    def updateParams(self,event):
        self.charge_time = rospy.get_param("~charge_time")
        self.drive_time = rospy.get_param("~drive_time")
        self.do_calib = rospy.get_param("~do_calib")
        self.charger = rospy.get_param("/maintenance_charger")


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[Charging Control Node] Shutdown.")
        if self.maintenance_state == "CHARGING":
            self.releaseChargerSpot()
            rospy.loginfo("[Charging Control Node] RELEASED CHARGER SPOT. PLEASE REMOVE DUCKIEBOT FROM CHARGER!!!")

    ##### END standard functions #####

if __name__ == '__main__':
    rospy.init_node('charging_control_node',anonymous=False)
    charging_control_node = ChargingControlNode()
    rospy.on_shutdown(charging_control_node.onShutdown)
    rospy.spin()
