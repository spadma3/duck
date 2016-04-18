#!/usr/bin/env python

import sys
import rospy
from navigation.srv import *
from duckietown_msgs.msg import FSMState, SourceTargetNodes, BoolStamped
from std_msgs.msg import Int16, String

class MissionPlannerNode():
    def __init__(self):
        self.node_name = "mission_planner_node"

        #adding logic because FSM publishes our state at a high rate
        #not just everytime the mode changes but multiple times in each mode
        self.first_update = True

        # Other state variables:
        self.actions = []
        self.fsm_mode = 'JOYSTICK_CONTROL'
        self.current_node = '0'
        self.localized = True
        self.path_in_progress = False

        # Parameters:
        self.trigger_mode = self.setupParameter("~trigger_mode","DECIDE_TURN_PATH_PLAN")
        self.use_localization = self.setupParameter("~use_localization", False)

        # Subscribers:
        self.sub_mode = rospy.Subscriber("~fsm_mode", FSMState, self.updateMode, queue_size = 1)
        self.sub_plan_request = rospy.Subscriber("~plan_request", SourceTargetNodes, self.graph_search)

        # Publishers:
        self.pub = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pubList = rospy.Publisher("~turn_plan", String, queue_size=1, latch=True)
        self.pub_localized = rospy.Publisher("~localized", BoolStamped, queue_size=1, latch=True)

        # Contingent upon use of localization:
        if self.use_localization:
            self.localized = False
        
            # --- ADD WHEN LOCALIZATION READY --- #
            # self.sub_location = rospy.Subscriber("~location,         cbLocalization

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updateMode(self, data):
        self.fsm_mode = data.state
        if (self.fsm_mode == "LOCALIZATION") and (not self.use_localization or self.path_in_progress):
            self.pubLocalized()
        self.dispatchTurn()

    def cbLocalization(self, data):

        # if localized -> set self.current_node = data.location and self.localized = True
        # if not localized -> set self.current_node = '0' and self.localized = True
        if not self.path_in_progress:
            self.pubLocalized()
        return

    def pubLocalized(self):
        msg = BoolStamped()
        msg.data = True
        self.pub_localized.publish(msg)

    def dispatchTurn(self):
        if self.first_update == False and self.fsm_mode != self.trigger_mode:
            self.first_update = True

        if self.first_update == True and self.fsm_mode == self.trigger_mode and self.actions:
            self.path_in_progress = True
            action = self.actions.pop(0)
            print 'Dispatched:', action
            if action == 's':
                self.pub.publish(Int16(1))
            elif action == 'r':
                self.pub.publish(Int16(2))
            elif action == 'l':
                self.pub.publish(Int16(0))
            elif action == 'w':
                self.path_in_progress = False
                self.pub.publish(Int16(-1))    
    
            action_str = ''
            for letter in self.actions:
                action_str += letter

            self.pubList.publish(action_str)
            self.firstUpdate = False

    def graph_search(self, data):
        if self.use_localization:
            print 'Requesting map for src: ', self.current_node, ' and target: ', data.target_node
        else:
            print 'Requesting map for src: ', data.source_node, ' and target: ', data.target_node
        rospy.wait_for_service('graph_search')
        try:
            graph_search = rospy.ServiceProxy('graph_search', GraphSearch)
            if self.use_localization:
                resp = graph_search(self.current_node, data.target_node)
            else:
                resp = graph_search(data.source_node, data.target_node)
            self.actions = resp.actions
            if self.actions:
                # remove 'f' (follow line) from actions and add wait action in the end of queue
                self.actions = [x for x in self.actions if x != 'f']
                self.actions.append('w')
                print 'Actions to be executed:', self.actions
                action_str = ''
                for letter in self.actions:
                    action_str += letter
                self.pubList.publish(action_str)
                self.dispatchTurn()
            else:
                print 'Actions to be executed:', self.actions

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def onShutdown(self):
        rospy.loginfo("[MissionPlannerNode] Shutdown.")

if __name__ == "__main__":
    rospy.init_node('mission_planner_node')
    mission_planner_node = MissionPlannerNode()
    rospy.on_shutdown(mission_planner_node.onShutdown)
    rospy.spin()
