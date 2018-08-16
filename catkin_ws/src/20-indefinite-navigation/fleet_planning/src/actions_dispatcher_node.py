#!/usr/bin/env python

import rospy
from fleet_planning.srv import *
from std_msgs.msg import Int16, ByteMultiArray
from duckietown_msgs.msg import BoolStamped
import numpy as np
import os
import tf
import tf2_ros
from fleet_planning.generate_duckietown_map import graph_creator
from fleet_planning.location_to_graph_mapping import IntersectionMapper
from fleet_planning.message_serialization import InstructionMessageSerializer, LocalizationMessageSerializer
from duckietown_msgs.msg import BoolStamped, FSMState, Twist2DStamped, MaintenanceState, TurnIDandType
from fleet_planning.duckiebot import TaxiEvent, NO_TARGET_LOCATION
# from rgb_led.srv import PlayLEDPattern


class ActionsDispatcherNode:
    _world_frame = 'world'
    _target_frame = 'duckiebot'

    def __init__(self):
        self.node_name = rospy.get_name()
        self.duckiebot_name = self.setup_parameter('/veh', 'no_duckiebot')
        map_dir = rospy.get_param('/map_dir')
        map_name = rospy.get_param('/map_name')

        self.actions = []
        self.path = []
        self.current_node = None
        self.target_node = rospy.get_param('end_node','261')
        self.last_red_line = rospy.get_time()
        self.active = False
        self.graphSearchSuccessful = False

        # Subscribers:
        self.sub_plan_request = rospy.Subscriber("~maintenance_state", MaintenanceState, self.cbMaintenanceState)
        # self.sub_red_line = rospy.Subscriber("~mode", FSMState, self.mode_update)
        self.sub_turn_type = rospy.Subscriber("~random_turn_id_and_type", TurnIDandType, self.cbTurnType)

        # Publishers:
        self.pub_action = rospy.Publisher("~turn_id_and_type", TurnIDandType, queue_size=1, latch=False)

        # location listener
        self.listener_transform = tf.TransformListener()

        # mapping: location -> node number
        self.graph_creator = graph_creator()
        self.graph_creator.build_graph_from_csv(map_dir, map_name)
        self.location_to_node_mapper = IntersectionMapper(self.graph_creator)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)

        rospy.set_param(param_name,value)  # Write to parameter server for transparency
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def cbMaintenanceState(self, msg):
        if msg.state == "WAY_TO_MAINTENANCE":
            self.active = True
            rospy.loginfo('[%s] ActionsDispatcherNode is active!' %(self.node_name))

        else:
            self.active = False

    def cbTurnType(self, msg):
        if not (self.graphSearchSuccessful and self.tag_id == msg.tag_id):
            self.graphSearchSuccessful = False

            self.tag_id = msg.tag_id
            self.turn_type = msg.turn_type

            if self.active == True:
                #self.update_current_node()
                #if self.current_node != None:
                self.current_node = self.tag_id
                self.graph_search(self.current_node, self.target_node)
                if self.graphSearchSuccessful == True:
                    self.dispatch_action(msg)
            else:
                self.pub_action.publish(msg)

        if self.current_node == self.target_node:
            rospy.loginfo('[%s] Destination reached!' %(self.node_name))

            self.actions = []
            #self.active = False

    def graph_search(self, source_node, target_node):
        rospy.loginfo('[%s] Requesting map for src: %s  and target: %s' %(self.node_name,repr(source_node),repr(target_node)))
        rospy.wait_for_service('graph_search')
        try:
            graph_search = rospy.ServiceProxy('graph_search', GraphSearch)
            resp = graph_search(str(source_node), str(target_node))

            self.path = resp.path
            actions = resp.actions

            if actions:
                # remove 'f' (follow line) from actions
                temp_actions = [x for x in actions if x != 'f']
                self.check_for_path_change(self.actions,temp_actions)
                self.actions = temp_actions
                info_msg = '\n \n ************ \n {} at node {} \n \n Actions to be executed: {}'.format(self.duckiebot_name, source_node, self.actions)
                rospy.loginfo('[%s] %s' %(self.node_name,info_msg))
                self.graphSearchSuccessful = True
            else:
                rospy.loginfo('[%s] No actions to be executed' %(self.node_name))

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def check_for_path_change(self, actions_old, actions_new):
        if actions_old == []:
            rospy.loginfo("Starting new route")
        else:
            act = actions_old.pop(0)
            if actions_old != actions_new:
                rospy.loginfo("Made a wrong turn! Planned turn %s", act)



    def dispatch_action(self,msg):
        if len(self.actions) > 0:
            action = self.actions.pop(0)
            action_name = None
            if action == 's':
                action_name = 'STRAIGHT'
                msg.turn_type = 1
                self.pub_action.publish(msg)
            elif action == 'r':
                action_name = 'RIGHT'
                msg.turn_type = 2
                self.pub_action.publish(msg)
            elif action == 'l':
                action_name = 'LEFT'
                msg.turn_type = 0
                self.pub_action.publish(msg)
            elif action == 'w':
                action_name = 'WAIT'
                msg.turn_type = -1
                self.pub_action.publish(msg)
            info_msg = 'Action: go {}!\n\n ************\n'.format(action_name)
            rospy.loginfo('[%s] %s' %(self.node_name,info_msg))

    def update_current_node(self):
        #TODO: map ID tags to node names
        rate = rospy.Rate(1.0)
        try:

            t = self.listener_transform.getLatestCommonTime(self._world_frame, self._target_frame)
            (trans, rot) = self.listener_transform.lookupTransform(self._world_frame, self._target_frame,
                                                                   rospy.Time(0))
            rot = tf.transformations.euler_from_quaternion(rot)[2]
            self.current_node = self.location_to_node_mapper.get_node_name(trans[:2], np.degrees(rot))
            info_msg = 'Current node:      ' + repr(self.current_node)
            rospy.loginfo('[%s] %s' %(self.node_name,info_msg))

        except tf2_ros.LookupException:
            rospy.logwarn('Duckiebot: {} location transform not found. Trying again.'.format(self.duckiebot_name))


    # def _play_led_pattern(self, pattern):
    #     play_pattern_service = rospy.ServiceProxy("/LEDPatternNode/play_pattern", PlayLEDPattern)
    #     try:
    #         response = play_pattern_service(pattern, 10)
    #         rospy.loginfo("Called play pattern service ({}). Got response: {}".format(pattern,response))
    #     except rospy.ServiceException as exc:
    #         rospy.logwarn("Call to play LED pattern service failed")
    #         rospy.logwarn(exc)

    def on_shutdown(self):
        rospy.loginfo("[ActionsDispatcherNode] Shutdown.")


if __name__ == "__main__":
    rospy.init_node('actions_dispatcher_node')

    actions_dispatcher_node = ActionsDispatcherNode()
    rospy.on_shutdown(actions_dispatcher_node.on_shutdown)
    rospy.spin()
