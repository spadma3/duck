#!/usr/bin/env python

from enum import Enum
import rospy
from fleet_planning.generate_duckietown_map import graph_creator
from std_msgs.msg import ByteMultiArray, String
from duckietown_msgs.msg import SourceTargetNodes
from fleet_planning.message_serialization import InstructionMessageSerializer, LocalizationMessageSerializer
from fleet_planning.duckiebot import *
import json
import random


class FleetPlanningStrategy(Enum): # for future expansion
    DEACTIVATED = 0
    CLOSEST_DUCKIEBOT = 1


class RebalancingStrategy(Enum):
    DEACTIVATED = 0
    RANDOM = 1


class TaxiCentralNode:
    TIME_OUT_CRITERIUM = 60.0
    _fleet_planning_strategy = FleetPlanningStrategy.CLOSEST_DUCKIEBOT # for now there is just this. gives room for future expansions
    _rebalancing_strategy = RebalancingStrategy.RANDOM

    def __init__(self):
        """
        subscribe to location", customer_requests. Publish to transportation status, target location.
        Init time_out timer.
        Specification see intermediate report document
        """
        self._registered_duckiebots = {}  # dict of instances of class Duckiebot. populated by register_duckiebot(). duckiebot name is key
        self._pending_customer_requests = []
        self._fulfilled_customer_requests = []  # for analysis purposes

        # graph classes for search and drawing
        gc = graph_creator()
        map_dir = rospy.get_param('/map_dir')
        map_name = rospy.get_param('/map_name')

        self._graph = gc.build_graph_from_csv(map_dir, map_name)
        self._graph_creator = gc

        # publishers
        self._pub_duckiebot_target_location = rospy.Publisher('/taxi/commands', ByteMultiArray, queue_size=1)
        self._pub_draw_command = rospy.Publisher("~/draw_request", String, queue_size=1, latch=True)

        # subscribers
        self._sub_customer_requests = rospy.Subscriber('~/customer_requests', SourceTargetNodes, self._register_customer_request, queue_size=1)
        self._sub_taxi_location = rospy.Subscriber('/taxi/location', ByteMultiArray, self._location_update)

        # timers
        self._time_out_timer = rospy.Timer(rospy.Duration.from_sec(self.TIME_OUT_CRITERIUM), self._check_time_out)

    @property
    def available_duckiebots(self):
        """
        :return: A list of all available _duckiebots.
        """
        return filter(lambda bot: bot.taxi_state == TaxiState.IDLE or bot.taxi_state == TaxiState.WITHOUT_MISSION,
                      self._registered_duckiebots.values())

    def _create_and_register_duckiebot(self, robot_name):
        """
        Whenever a new duckiebot is detected, this method is called. Create Duckiebot instance and append to _registered_duckiebots
        E.g. an unknown duckiebot publishes a location -> register duckiebot
        :param robot_name: string
        """
        duckiebot = Duckiebot(robot_name)
        if robot_name not in self._registered_duckiebots:
            self._registered_duckiebots[robot_name] = duckiebot
            rospy.loginfo('Created and registered Duckiebot {}'.format(robot_name))

        else:
            rospy.logwarn('Failed to register new duckiebot. A duckiebot with the same name has already been registered.')

    def _unregister_duckiebot(self, duckiebot):
        """unregister given duckiebot, remove from map drawing. If it currently has been assigned a customer,
        put customer request back to _pending_customer_requests"""

        request = duckiebot.pop_customer_request()
        if request is not None:
            self._pending_customer_requests[:0] = [request]  # prepend, high priority

        try:
            del self._registered_duckiebots[duckiebot.name]
            rospy.logwarn('Unregistered and removed from map Duckiebot {}'.format(duckiebot.name))
        except KeyError:
            rospy.logwarn('Failure when unregistering Duckiebot. {} had already been unregistered.'.format(duckiebot.name))

        self._publish_draw_request()

    def _register_customer_request(self, request_msg):
        """callback function for request subscriber. appends CustomerRequest instance to _pending_customer_requests,
        Calls handle_customer_requests

        """
        start = request_msg.source_node
        target = request_msg.target_node
        request = CustomerRequest(start, target)
        self._pending_customer_requests.append(request)
        rospy.loginfo('Registered customer request {} -> {}'.format(start, target))
        self._publish_draw_request()
        self._handle_customer_requests()

    def _execute_fleet_planning(self):
        self._handle_customer_requests()
        self._rebalance()

    def _handle_customer_requests(self):
        """
        Switch function. This allows to switch between strategies in the future
        """
        if self._fleet_planning_strategy == FleetPlanningStrategy.CLOSEST_DUCKIEBOT:
            self._fleet_planning_closest_duckiebot()

        elif self._fleet_planning_strategy == FleetPlanningStrategy.DEACTIVATED:
            # do nothing. used mainly for unit tests
            pass
        else:
            raise NotImplementedError('Chosen strategy has not yet been implemented.')

    def _fleet_planning_closest_duckiebot(self):
        """
        E.g. for every pending customer request do breadth first search to find closest available duckiebot.
        Make sure to use Duckiebot.next_location for the search. Finally assign customer request to best duckiebot.
        """
        # For now quickly find the closest duckiebot

        # We work on the inverted graph. Because we want the duckiebot that has the shortest
        # path to the location. I.e. if the duckiebot already drove past the location it
        # might have to go a long distance. We don't want that.
        graph = self._graph.get_inverted_graph()

        while len(self._pending_customer_requests) > 0:
            pending_request = self._pending_customer_requests[0]
            available_duckiebots = self.available_duckiebots
            if len(available_duckiebots) == 0:
                rospy.loginfo("No duckiebot available for pending transport request")
                return

            # Get the start node
            start_node = graph.get_node(pending_request.start_location)

            nodes_to_visit = [start_node]
            visited_nodes = []

            # Find the closest duckiebot via breadth first search
            duckiebot = None
            while len(nodes_to_visit) > 0 and duckiebot is None:
                current_node = nodes_to_visit.pop(0)
                visited_nodes.append(current_node)

                # Check if there's a duckiebot on that node
                for db in available_duckiebots:
                    if db.next_location == '-1':  # if duckiebot has no mission, it is not moving. use current node
                        db_location = db.location
                    else:
                        db_location = db.next_location # if duckiebot has mission, location is probably outdated, use next expected location

                    if str(db_location) == current_node:
                        # We found one!
                        duckiebot = db
                        break

                # Add all the neighboring nodes to the list of nodes we still have to visit
                edges = graph.node_edges(current_node)
                for edge in edges:
                    if str(edge.target) not in visited_nodes:
                        nodes_to_visit.append(edge.target)

            if duckiebot is not None:
                # Assign the request to that duckiebot
                self._pending_customer_requests.pop(0)
                duckiebot.assign_customer_request(pending_request)
                self._publish_duckiebot_mission(duckiebot, TaxiEvent.ACCEPTED_REQUEST)
            else:
                rospy.logwarn("There are available _duckiebots but they were not found in the graph. Aborting assignment procedure.")
                break

    def _location_update(self, message):
        """
        Callback function for location subscriber. Message contains location and robot name.  If duckiebot
        is not yet known, register it first. Location is first mapped from 2d coordinates to graph node, then call
        Duckiebot.update_location_check_target_reached(..). According to its feedback move customer request to
        _fulfilled_customer_requests. If taxi has become free, call handle_customer_requests
        Update map drawing correspondingly (taxi location, customer location). Publish duckiebot taxi state
         if it has changed.
        :param location_msg: contains location and robot name
        """
        duckiebot_name, node, route = LocalizationMessageSerializer.deserialize("".join(map(chr, message.data)))

        # The whole taxi_central_node uses strings as node ids. So let's make sure we use strings too
        # from this point onwards.
        node = str(node)
        route = map(str, route)
        taxi_event = None

        if duckiebot_name not in self._registered_duckiebots: # new duckiebot
            self._create_and_register_duckiebot(duckiebot_name)
            duckiebot = self._registered_duckiebots[duckiebot_name]
            new_duckiebot_state, taxi_event = duckiebot.update_location_check_target_reached(node, route)

        else:
            duckiebot = self._registered_duckiebots[duckiebot_name]
            new_duckiebot_state, taxi_event = duckiebot.update_location_check_target_reached(node, route)
            if taxi_event == TaxiEvent.DROPOFF_CUSTOMER:
                self._publish_duckiebot_mission(duckiebot, taxi_event)

        if new_duckiebot_state == TaxiState.WITHOUT_MISSION: # mission accomplished or without mission
            request = duckiebot.pop_customer_request()
            if request is not None:
                self._fulfilled_customer_requests.append(request)

            self._execute_fleet_planning()

        elif new_duckiebot_state == TaxiState.WITH_CUSTOMER: # reached customer
            self._publish_duckiebot_mission(duckiebot, taxi_event) # make duckiebot go to customer target location

        else: # nothing special happened, just location update
            pass
        self._publish_draw_request()

    def _rebalance(self):
        if self._rebalancing_strategy == RebalancingStrategy.RANDOM:
            self.random_rebalancing()

    def random_rebalancing(self):
        all_possible_nodes = self._graph.intersection_nodes

        for duckiebot in self._registered_duckiebots.values():
            if duckiebot.taxi_state == TaxiState.WITHOUT_MISSION:

                repeat = True
                while repeat:
                    target = random.choice(all_possible_nodes)
                    if target != duckiebot.location:
                        repeat = False

                duckiebot.assign_rebalancing_mission(target)
                rospy.loginfo('Rebalancing Duckiebot {}'.format(duckiebot.name))
                self._publish_duckiebot_mission(duckiebot, TaxiEvent.NONE_EVENT)

    def _check_time_out(self, msg):
        """callback function from some timer, ie. every 30 seconds. Checks for every duckiebot whether it has been
        seen since the last check_time_out call. If not, unregister duckiebot"""

        for duckiebot in self._registered_duckiebots.values():
            if duckiebot.has_timed_out(self.TIME_OUT_CRITERIUM):
                rospy.logwarn('Duckiebot {} has timed out.'.format(duckiebot.name))
                self._unregister_duckiebot(duckiebot)

    def _publish_duckiebot_mission(self, duckiebot, taxi_event):
        """ create message that sends duckiebot to its next location, according to the customer request that had been
        assigned to it"""
        serialized_message = InstructionMessageSerializer.serialize(duckiebot.name, int(duckiebot.target_location),
                                                                    taxi_event.value)
        self._pub_duckiebot_target_location.publish(ByteMultiArray(data=serialized_message))

        rospy.loginfo('Duckiebot {} was sent to node {}'.format(duckiebot.name, duckiebot.target_location))

    def _publish_draw_request(self):
        dict = {'duckiebots': [db[1].to_dict() for db in self._registered_duckiebots.items()],
                'pending_customer_requests': [cr.to_dict() for cr in self._pending_customer_requests if cr is not None]}
        self._pub_draw_command.publish(json.dumps(dict))

    def save_metrics(self): # implementation has rather low priority
        """ gather timestamps from customer requests, calculate metrics, save to json file"""
        pass

    @staticmethod
    def on_shutdown():
        rospy.loginfo("[TaxiCentralNode] Shutdown.")


if __name__ == '__main__':
    # startup node
    rospy.loginfo('[taxi_central_node]: Startup.')
    rospy.init_node('taxi_central_node')
    taxi_central_node = TaxiCentralNode()

    rospy.on_shutdown(TaxiCentralNode.on_shutdown)
    rospy.spin()
