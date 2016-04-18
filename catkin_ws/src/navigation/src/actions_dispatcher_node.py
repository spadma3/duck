#!/usr/bin/env python

import sys
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped
from std_msgs.msg import Int16, String

class ActionsDispatcherNode():
    def __init__(self):
        self.node_name = "actions_dispatcher_node"

        #adding logic because FSM publishes our state at a high rate
        #not just everytime the mode changes but multiple times in each mode
        self.first_update = True

        self.next_turn = -1
        self.random_turn = -2
        self.path_turn = -2
        self.fsm_mode = 'JOYSTICK_CONTROL'

        # Parameters:
        self.random_trigger_mode = self.setupParameter("~random_trigger_mode","DECIDE_TURN_RANDOM")
        self.mission_trigger_mode = self.setupParameter("~mission_trigger_mode","DECIDE_TURN_PATH_PLAN")
        self.self_trigger_mode = self.setupParameter("~self_trigger_mode","INTERSECTION_CONTROL")

        # Subscribers:
        self.sub_mode = rospy.Subscriber("~fsm_mode", FSMState, self.updateMode, queue_size = 1)
        self.sub_random_turn_type = rospy.Subscriber("~random_turn_type", Int16, self.updateTurnTypeRandom)
        self.sub_plan_turn_type = rospy.Subscriber("~plan_turn_type", Int16, self.updateTurnTypePlan)

        # Publishers:
        self.pub = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_turn_decided = rospy.Publisher("~turn_decided", BoolStamped, queue_size=1, latch=True)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updateMode(self, data):
        self.fsm_mode = data.state
        self.updateNextTurn()
        self.dispatcher()

    def updateTurnTypeRandom(self, msg):
        self.random_turn = msg.data
        self.updateNextTurn()

    def updateTurnTypePlan(self, msg):
        self.path_turn = msg.data
        self.updateNextTurn()

    def updateNextTurn(self):
        if self.fsm_mode == self.random_trigger_mode and self.random_turn != -2:
            self.next_turn = self.random_turn
            self.dispatcher()
            self.pubTurnDecided()
        elif self.fsm_mode == self.mission_trigger_mode and self.path_turn != -2:
            self.next_turn = self.path_turn
            self.dispatcher()
            self.pubTurnDecided()

    def pubTurnDecided(self):
        msg = BoolStamped()
        msg.data = True
        self.pub_turn_decided.publish(msg)

    def dispatcher(self):
        if self.first_update == False and self.fsm_mode != self.self_trigger_mode:
            self.first_update = True

        if self.first_update == True and self.fsm_mode == self.self_trigger_mode:
            # Allow time for open loop controller to update state and allow duckiebot to stop at redline:
            rospy.sleep(2)
        
            # Proceed with action dispatching:
            print 'Dispatched:', self.next_turn
            self.pub.publish(Int16(self.next_turn))
            self.firstUpdate = False
            self.random_turn = -2
            self.path_turn = -2
        elif self.fsm_mode != self.self_trigger_mode:
            self.pub.publish(Int16(-1))

    def onShutdown(self):
        rospy.loginfo("[ActionsDispatcherNode] Shutdown.")

if __name__ == "__main__":
    rospy.init_node('actions_dispatcher_node')
    actions_dispatcher_node = ActionsDispatcherNode()
    rospy.on_shutdown(actions_dispatcher_node.onShutdown)
    rospy.spin()
