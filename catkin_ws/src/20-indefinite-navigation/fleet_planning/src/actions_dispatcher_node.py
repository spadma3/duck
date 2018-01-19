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
from duckietown_msgs.msg import BoolStamped, FSMState, Twist2DStamped

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
        self.target_node = None
        self.last_red_line = rospy.get_time()

        # Subscribers:
        self.sub_plan_request = rospy.Subscriber("~/taxi/commands", ByteMultiArray, self.new_duckiebot_mission)
        self.sub_red_line = rospy.Subscriber("~mode", FSMState, self.mode_update)


        # location listener
        self.listener_transform = tf.TransformListener()

        # Publishers:
        self.pub_action = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_location_node = rospy.Publisher("/taxi/location", ByteMultiArray, queue_size=1)
        self.pub_intersection_go = rospy.Publisher('simple_coordinator_node/intersection_go', BoolStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher('simple_coordinator_node/car_cmd', Twist2DStamped, queue_size=1)

        # mapping: location -> node number
        self.graph_creator = graph_creator()
        self.graph_creator.build_graph_from_csv(map_dir, map_name)
        self.location_to_node_mapper = IntersectionMapper(self.graph_creator)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)

        rospy.set_param(param_name,value)  # Write to parameter server for transparency
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def mode_update(self, msg):
        rospy.logwarn("mode update action dispatcher {}".format(msg.state))
        if msg.state == "COORDINATION":
            rospy.logwarn("CORDINATION ACTION DISPATCHER")
            self.pub_car_cmd.publish(Twist2DStamped(v=0, omega=0))
            self.localize_at_red_line(msg)

    def localize_at_red_line(self, msg):
        rospy.loginfo('Localizing.')

        start_time = rospy.get_time()
        node = None
        rate = rospy.Rate(1.0)
        while not node and rospy.get_time() - start_time < 5.0:  # TODO: tune this

            try:
                (trans, rot) = self.listener_transform.lookupTransform(self._world_frame, self._target_frame,
                                                                       rospy.Time(0))
                rot = tf.transformations.euler_from_quaternion(rot)[2]
                node = self.location_to_node_mapper.get_node_name(trans[:2], np.degrees(rot))

            except tf2_ros.LookupException:
                rospy.logwarn('Duckiebot: {} location transform not found. Trying again.'.format(self.duckiebot_name))

            if not node:
                rate.sleep()

        if not node:
            rospy.logwarn('Duckiebot: {} location update failed. Location not updated.'.format(self.duckiebot_name))
            return

        node = int(node)
        rospy.loginfo('Duckiebot {} located at node {}'.format(self.duckiebot_name, node))

        location_message = LocalizationMessageSerializer.serialize(self.duckiebot_name, node, self.path)
        self.pub_location_node.publish(ByteMultiArray(data=location_message))

        if self.target_node is None or self.target_node == node:
            rate_recursion = rospy.Rate(0.5)
            rate_recursion.sleep()
            self.localize_at_red_line(msg) # repeat until new duckiebot mission was published # TODO: improve this?

        else:
            self.graph_search(node, self.target_node)
            self.pub_intersection_go.publish(BoolStamped(header=msg.header, data=True))
            self.dispatch_action()
            location_message = LocalizationMessageSerializer.serialize(self.duckiebot_name, node, self.path)
            self.pub_location_node.publish(ByteMultiArray(data=location_message))

    def new_duckiebot_mission(self, message):
        duckiebot_name, target_node, taxi_state = InstructionMessageSerializer.deserialize("".join(map(chr, message.data)))
        if duckiebot_name != self.duckiebot_name:
            return
        self.target_node = target_node

    def graph_search(self, source_node, target_node):
        print 'Requesting map for src: ', source_node, ' and target: ', target_node
        rospy.wait_for_service('graph_search')
        try:
            graph_search = rospy.ServiceProxy('graph_search', GraphSearch)
            resp = graph_search(str(source_node), str(target_node))

            self.path = resp.path
            actions = resp.actions

            if actions:
                # remove 'f' (follow line) from actions
                self.actions = [x for x in actions if x != 'f']
                print '\n \n ************ \n {} at node {} \n \n Actions to be executed: {}'.format(self.duckiebot_name, source_node, self.actions)
            else:
                print 'No actions to be executed'

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def dispatch_action(self):
        if len(self.actions) > 0:
            action = self.actions.pop(0)
            action_name = None
            if action == 's':
                action_name = 'STRAIGHT'
                self.pub_action.publish(Int16(1))
            elif action == 'r':
                action_name = 'RIGHT'
                self.pub_action.publish(Int16(2))
            elif action == 'l':
                action_name = 'LEFT'
                self.pub_action.publish(Int16(0))
            elif action == 'w':
                action_name = 'WAIT'
                self.pub_action.publish(Int16(-1))
            print 'Action: go {}!\n\n ************\n'.format(action_name)

    def on_shutdown(self):
        rospy.loginfo("[ActionsDispatcherNode] Shutdown.")


if __name__ == "__main__":
    rospy.init_node('actions_dispatcher_node')

    actions_dispatcher_node = ActionsDispatcherNode()
    rospy.on_shutdown(actions_dispatcher_node.on_shutdown)
    rospy.spin()
