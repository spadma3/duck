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


class ActionsDispatcherNode:
    _world_frame = 'world'
    _target_frame = 'duckiebot'

    def __init__(self, map_dir, map_csv):
        self.node_name = rospy.get_name()
        self.duckiebot_name = self.setup_parameter('/veh', 'no_duckiebot')
        map_dir = rospy.get_param('/map_dir')
        map_name = rospy.get_param('/map_name')

        self.actions = []
        self.target_node = None
        self.last_red_line = rospy.get_time()

        # Subscribers:
        self.sub_plan_request = rospy.Subscriber("~/taxi/commands", ByteMultiArray, self.graph_search)
        self.sub_red_line = rospy.Subscriber("~/paco/stop_line_filter_node/at_stop_line", BoolStamped, self.localize_at_red_line)

        # location listener
        self.listener_transform = tf.TransformListener()

        # Publishers:
        self.pub_action = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
        self.pub_location_node = rospy.Publisher("/taxi/location", ByteMultiArray, queue_size=1)

        # mapping: location -> node number
        self.graph_creator = graph_creator()
        self.graph_creator.build_graph_from_csv(map_dir, map_name)
        self.location_to_node_mapper = IntersectionMapper(self.graph_creator)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)

        rospy.set_param(param_name,value)  # Write to parameter server for transparency
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def localize_at_red_line(self, message):
        if rospy.get_time() - self.last_red_line < 5.0 and message is not None:  # time out filter in case that this is triggered more than once at intersection. very suboptimal
            rospy.logwarn('Location not updated, red line too soon detected after last one.')
            return
        self.last_red_line = rospy.get_time()

        rospy.loginfo('Localizing.')

        start_time = rospy.get_time()
        node = None
        rate = rospy.Rate(5)
        while not node and rospy.get_time() - start_time < 5.0:  # TODO: tune this

            try:
                (trans, rot) = self.listener_transform.lookupTransform(self._world_frame, self._target_frame,
                                                                       rospy.Time(0))
                rot = tf.transformations.euler_from_quaternion(rot)[2]
                node = self.location_to_node_mapper.get_node_name(trans[:2], np.degrees(rot))

            except tf2_ros.LookupException:
                rospy.logwarn('Duckiebot: {} location transform not found. Trying again.'.format(self.duckiebot_name))

            rate.sleep()

        if not node:
            rospy.logwarn('Duckiebot: {} location update failed. Location not updated.'.format(self.duckiebot_name))
            return

        node = int(node)
        rospy.loginfo('Duckiebot {} located at node {}'.format(self.duckiebot_name, node))

        location_message = LocalizationMessageSerializer.serialize(self.duckiebot_name, node)
        self.pub_location_node.publish(ByteMultiArray, location_message)

        if self.target_node is None or self.target_node == node:
            self.localize_at_red_line(None) # repeat until new duckiebot mission was published

        else:
            self.graph_search(node, self.target_node)
            self.dispatch_action()

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
            actions = resp.actions

            if actions:
                # remove 'f' (follow line) from actions
                self.actions = [x for x in actions if x != 'f']
                print 'Actions to be executed:', self.actions
            else:
                print 'No actions to be executed'

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def dispatch_action(self):
        if len(self.actions) > 0:
            action = self.actions.pop(0)
            print 'Dispatched action:', action
            if action == 's':
                self.pub_action.publish(Int16(1))
            elif action == 'r':
                self.pub_action.publish(Int16(2))
            elif action == 'l':
                self.pub_action.publish(Int16(0))
            elif action == 'w':
                self.pub_action.publish(Int16(-1))

    def on_shutdown(self):
        rospy.loginfo("[ActionsDispatcherNode] Shutdown.")


if __name__ == "__main__":
    rospy.init_node('actions_dispatcher_node')

    actions_dispatcher_node = ActionsDispatcherNode()
    rospy.on_shutdown(actions_dispatcher_node.on_shutdown)
    rospy.spin()
