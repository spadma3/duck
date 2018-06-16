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

        self.maintenance_state = "NONE"
        self.inMaintenanceArea = False

        self.state = "JOYSTICK_CONTROL"

        self.tag_id = -1
        self.turn_type = -1

        ## Subscribers
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.inters_done = rospy.Subscriber("~intersection_done", BoolStamped, self.cbIntersecDone)

        self.go_charging = rospy.Subscriber("~go_charging", Bool, self.cbGoCharging)
        self.go_calibrating = rospy.Subscriber("~go_calibrating", Bool, self.cbGoCalibrating)
        self.set_state = rospy.Subscriber("~set_state", String, self.cbSetState)

        ## Publisher
        self.pub_maintenance_state = rospy.Publisher("~maintenance_state", MaintenanceState, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)



    def cbSetState(self, msg):
        self.maintenance_state = msg.data
        self.pubMaintenanceState()
        
    def cbTurnType(self, msg):
        self.tag_id = msg.tag_id
        self.turn_type = msg.turn_type

    def cbGoCharging(self, msg):
        if msg.data:
            self.maintenance_state = "WAY_TO_CHARGING"
            self.pubMaintenanceState()
            rospy.loginfo("[Maintenance Control Node] State: WAY_TO_CHARGING")

    def cbGoCalibrating(self, msg):
        if msg.data:
            self.maintenance_state = "WAY_TO_CALIBRATING"
            self.pubMaintenanceState()
            rospy.loginfo("[Maintenance Control Node] State: WAY_TO_CALIBRATING")



    # Executes when intersection is done
    def cbIntersecDone(self, msg):
        turn = [self.tag_id, self.turn_type]

        # Entering maintenance area
        if turn in self.maintenance_entrance:
            self.inMaintenanceArea = True
            maintenance_msg = MaintenanceState()
            if self.maintenance_state == "WAY_TO_CHARGING":
                self.maintenance_state = "CHARGING"
                self.pubMaintenanceState()

            if self.maintenance_state == "WAY_TO_CALIBRATING":
                self.maintenance_state = "CALIBRATING"
                self.pubMaintenanceState()

            rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            rospy.loginfo("Entering maintenance area!")
            rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

        # Leaving maintenance area
        if turn in self.maintenance_exit:
            self.inMaintenanceArea = False
            self.maintenance_state = "NONE"
            self.pubMaintenanceState()
            rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            rospy.loginfo("Leaving maintenance area!")
            rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")



    def pubMaintenanceState(self):
        maintenance_msg = MaintenanceState()
        maintenance_msg.state = self.maintenance_state
        self.pub_maintenance_state.publish(maintenance_msg)

    # Returns the turn type for an intersection to get to charger
    def getTurnType(self, chargerID, tagID):
        station = self.stations['station' + str(chargerID)]
        turn = -1
        for el in station.path_in:
            if el[0] == tagID:
                turn = el[1]
        for el in station.path_out:
            if el[0] == tagID:
                turn = el[1]
        return turn


    def setupParams(self):
        self.maintenance_entrance = self.setupParam("~maintenance_entrance", 0)
        self.maintenance_exit = self.setupParam("~maintenance_exit", 0)


    def updateParams(self,event):
        self.maintenance_entrance = rospy.get_param("~maintenance_entrance")
        self.maintenance_exit = rospy.get_param("~maintenance_exit")




    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[MaintenanceControlNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('maintenance_control_node',anonymous=False)
    maintenance_control_node = MaintenanceControlNode()
    rospy.on_shutdown(maintenance_control_node.onShutdown)
    rospy.spin()
