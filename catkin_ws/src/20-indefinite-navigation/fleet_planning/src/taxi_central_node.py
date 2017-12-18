#!/usr/bin/env python

from enum import Enum, IntEnum
import os
import rospy
from fleet_planning.generate_duckietown_map import graph_creator
from std_msgs.msg import ByteMultiArray
from duckietown_msgs.msg import SourceTargetNodes
from fleet_planning.message_serialization import InstructionMessageSerializer, LocalizationMessageSerializer
from fleet_planning.map_drawing import MapDraw
from sensor_msgs.msg import Image

class TaxiState(IntEnum):
    GOING_TO_CUSTOMER = 0
    WITH_CUSTOMER = 1
    IDLE = 2


class Instruction(Enum):
    LEFT = 'l'
    RIGHT = 'r'
    STRAIGHT = 's'


class FleetPlanningStrategy(Enum): # for future expansion
    DEACTIVATED = 0
    CLOSEST_DUCKIEBOT = 1


class Duckiebot:
    """tracks state and mission of one duckiebot, handles the global customer and location assignments"""

    def __init__(self, robot_name):
        self._name = robot_name
        self._taxi_state = TaxiState.IDLE

        self._last_known_location = None  # number of the node the localization lastly reported
        self._next_expected_location = None
        self._last_time_seen_alive = None  # timestamp. updated every time a location or similar was reported. Duckiebot is removed from map if this becomes too far in the past

        self._target_location = None
        self._customer_request = None  # instance of CustomerRequest, only not None if on duty

    @property
    def taxi_state(self):
        return self._taxi_state

    @property
    def name(self):
        return self._name

    @property
    def customer_request(self):
        return self._customer_request

    @property
    def location(self): # This can be adapted to take into account some between-intersections location estimation
        """
        :return: node number of last known location of duckiebot
        """
        return self._last_known_location

    @property
    def next_location(self):
        """
        :return: node number of expected next duckiebot location
        """
        return self._next_expected_location

    @property
    def target_location(self):
        """returns target location of Duckiebots current mission,
        depending on the status of the customer request it is handling."""

        if self._taxi_state == TaxiState.IDLE:
            return -1

        if self.taxi_state == TaxiState.GOING_TO_CUSTOMER:
            return self._customer_request.start_location

        if self.taxi_state == TaxiState.WITH_CUSTOMER:
            return self._customer_request.target_location

    def update_location_check_target_reached(self, reported_location, next_location):
        """
        updates member _last_known_location. If duckiebot is now at target location and has a customer request,
        it checks whether current location is customer start location or customer target location.
        It updates its _taxi_state correspondingly and updates sets _last_time_seen_alive and CustomerRequest time stamps.
        :param reported_location: reported from localization
        :param next_location: where duckiebot is expected to show up next
        :return: None if status has not changed, or duckiebot is new. Returns self_taxi state if customer has been
                picked up or customer target location has been reached.
        """
        if reported_location is None:
            return None

        self._last_known_location = reported_location
        self._next_expected_location = next_location
        self._last_time_seen_alive = rospy.get_time()

        if self._customer_request is not None:
            if reported_location == self._customer_request.start_location:
                self._taxi_state = TaxiState.WITH_CUSTOMER
                self._customer_request.time_pickup = rospy.get_time()
                return self._taxi_state

            elif reported_location == self._customer_request.target_location:
                self._taxi_state = TaxiState.IDLE
                self._customer_request.time_drop_off = rospy.get_time()
                return self._taxi_state

            else:
                return None
        else:
            return None

    def has_timed_out(self, criterium):
        if rospy.get_time() - self._last_time_seen_alive > criterium:
            return True
        else:
            return False

    def assign_customer_request(self, customer_request):
        """ assign customer request to this duckiebot"""
        if self._customer_request is not None:
            raise ValueError('Forbidden customer assignment. This Duckiebot has beed assigned a customer already.')

        self._customer_request = customer_request
        self._taxi_state = TaxiState.GOING_TO_CUSTOMER

    def pop_customer_request(self):
        self._taxi_state = TaxiState.IDLE

        tmp = self._customer_request
        self._customer_request = None
        return tmp

        
class CustomerRequest:

    def __init__(self, start_node, target_node):
        self.start_location = start_node # node number
        self.target_location = target_node # node number

        # for the metrics. Use ropy.time() to set timestamp
        self.time_registered = rospy.get_time()
        self.time_pickup = None
        self.time_drop_off = None


class TaxiCentralNode:
    TIME_OUT_CRITERIUM = 60.0
    _fleet_planning_strategy = FleetPlanningStrategy.CLOSEST_DUCKIEBOT # for now there is just this. gives room for future expansions

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
        gui_img_dir = rospy.get_param('/gui_img_dir')
        self._graph = gc.build_graph_from_csv(map_dir, map_name)
        self._graph_creator = gc

        # subscribers
        self._sub_customer_requests = rospy.Subscriber('~/customer_requests', SourceTargetNodes,
                                                       self._register_customer_request, queue_size=1)
        self._sub_taxi_location = rospy.Subscriber('/taxi/location', ByteMultiArray, self._location_update)

        # publishers
        self._pub_duckiebot_target_location = rospy.Publisher('/taxi/commands', ByteMultiArray, queue_size=1)
        self.image_pub = rospy.Publisher("~map_graph", Image, queue_size=1, latch=True)

        # timers
        self._time_out_timer = rospy.Timer(rospy.Duration.from_sec(self.TIME_OUT_CRITERIUM), self._check_time_out)

        # map drawing
        self.map_drawing = MapDraw(self._graph, map_dir, gui_img_dir, map_name)
        self.image_pub.publish(self.map_drawing.drawMap({}))

    def _idle_duckiebots(self):
        """
        :return: A list of all IDLE duckiebots.
        """
        return filter(lambda bot: bot.taxi_state == TaxiState.IDLE, self._registered_duckiebots.values())

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

        request = duckiebot.pop_customer_request
        if request is not None:
            self._pending_customer_requests[:0] = [duckiebot.pop_customer_request()]  # prepend, high priority

        try:
            del self._registered_duckiebots[duckiebot.name]
            rospy.logwarn('Unregistered and removed from map Duckiebot {}'.format(duckiebot.name))
        except KeyError:
            rospy.logwarn('Failure when unregistering Duckiebot. {} had already been unregistered.'.format(duckiebot.name))

        self.image_pub.publish(self.map_drawing.drawMap(self._registered_duckiebots))

    def _register_customer_request(self, request_msg):
        """callback function for request subscriber. appends CustomerRequest instance to _pending_customer_requests,
        Calls handle_customer_requests

        """
        start = request_msg.source_node
        target = request_msg.target_node
        request = CustomerRequest(start, target)
        self._pending_customer_requests.append(request)
        rospy.loginfo('Registered customer request {} -> {}'.format(start, target))
        self._handle_customer_requests()

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
        E.g. for every pending customer request do breadth first search to find closest idle duckiebot.
        Make sure to use Duckiebot.next_location for the search. Finally assign customer request to best duckiebot.
        (Maybe if # pending_customer requests > number idle duckiebots, assign the ones with the shortest path.)
        """
        # For now quickly find the closest duckiebot
        rospy.logwarn(self._pending_customer_requests)
        while len(self._pending_customer_requests) > 0:
            pending_request = self._pending_customer_requests[0]
            idle_duckiebots = self._idle_duckiebots()
            if len(idle_duckiebots) == 0:
                rospy.loginfo("No duckiebot available for pending transport request")
                return

            # Get the start node
            start_node = self._graph.get_node(pending_request.start_location)

            nodes_to_visit = [start_node]
            visited_nodes = []

            # Find the closest duckiebot via breadth first search
            duckiebot = None
            while len(nodes_to_visit) > 0 and duckiebot is None:
                current_node = nodes_to_visit.pop(0)
                visited_nodes.append(current_node)

                # Check if there's a duckiebot on that node
                for db in idle_duckiebots:
                    if str(db.next_location) == current_node:
                        # We found one!
                        duckiebot = db
                        break

                # Add all the neighboring nodes to the list of nodes we still have to visit
                edges = self._graph.node_edges(current_node)
                for edge in edges:
                    if str(edge.target) not in visited_nodes:
                        nodes_to_visit.append(edge.target)

            if duckiebot is not None:
                # Assign the request to that duckiebot
                self._pending_customer_requests.pop(0)
                duckiebot.assign_customer_request(pending_request)
                self._publish_duckiebot_mission(duckiebot)
            else:
                rospy.logwarn("There are IDLE duckiebots but they were not found in the graph")

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
        rospy.logwarn(duckiebot_name)
        rospy.logwarn(node)
        rospy.logwarn(route)
        # Find the next node
        next_node = -1
        for n in range(len(route)):
            if route[n] == node:
                if n + 1 < len(route):
                    next_node = route[n+1]
                    break

        # The whole taxi_central_node uses strings as node ids. So let's make sure we use strings too
        # from this point onwards.
        node = str(node)
        next_node = str(next_node)

        if duckiebot_name not in self._registered_duckiebots:
            self._create_and_register_duckiebot(duckiebot_name)
            duckiebot = self._registered_duckiebots[duckiebot_name]
            new_duckiebot_state = duckiebot.update_location_check_target_reached(node, next_node)
            self._handle_customer_requests()

        else:
            duckiebot = self._registered_duckiebots[duckiebot_name]
            new_duckiebot_state = duckiebot.update_location_check_target_reached(node, next_node)

        if new_duckiebot_state == TaxiState.IDLE: # mission accomplished
            rospy.loginfo('Duckiebot {} has dropped off its happy customer.'.format(duckiebot.name))
            request = duckiebot.pop_customer_request()
            self._fulfilled_customer_requests.append(request)
            self._handle_customer_requests() # bcs duckiebot is available again
            self._publish_duckiebot_mission(duckiebot)
            # TODO raise flag to make duckiebot go around randomly. or create random targets

        elif new_duckiebot_state == TaxiState.WITH_CUSTOMER: # reached customer
            rospy.loginfo('Duckiebot {} has reached its customer.'.format(duckiebot.name))
            self._publish_duckiebot_mission(duckiebot)

        else: # nothing special happened, just location update
            pass
        self.image_pub.publish(self.map_drawing.drawMap(self._registered_duckiebots))

    def _check_time_out(self, msg):
        """callback function from some timer, ie. every 30 seconds. Checks for every duckiebot whether it has been
        seen since the last check_time_out call. If not, unregister duckiebot"""

        for duckiebot in self._registered_duckiebots.values():
            if duckiebot.has_timed_out(self.TIME_OUT_CRITERIUM):
                rospy.logwarn('Duckiebot {} has timed out.'.format(duckiebot.name))
                self._unregister_duckiebot(duckiebot)

    def _publish_duckiebot_mission(self, duckiebot):
        """ create message that sends duckiebot to its next location, according to the customer request that had been
        assigned to it"""
        rospy.loginfo('location:{}'.format(duckiebot.target_location))
        rospy.loginfo('taxi state {}'.format(duckiebot.taxi_state.value))
        rospy.loginfo(type(duckiebot.target_location))
        rospy.loginfo(type(duckiebot.taxi_state.value))
        serialized_message = InstructionMessageSerializer.serialize(duckiebot.name, int(duckiebot.target_location),
                                                                    duckiebot.taxi_state.value)
        self._pub_duckiebot_target_location.publish(ByteMultiArray(data=serialized_message))

        rospy.loginfo('Duckiebot {} was sent to node {}'.format(duckiebot.name, duckiebot.target_location))

    def save_metrics(self): # implementation has rather low priority
        """ gather timestamps from customer requests, calculate metrics, save to json file"""
        pass

    @staticmethod
    def on_shutdown():
        rospy.loginfo("[TaxiCentralNode] Shutdown.")


if __name__ == '__main__':
    # startup node
    rospy.init_node('taxi_central_node')
    taxi_central_node = TaxiCentralNode()

    rospy.on_shutdown(TaxiCentralNode.on_shutdown)
    rospy.spin()
