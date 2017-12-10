#!/usr/bin/env python

from enum import Enum
import os
import rospy
import cv2
import numpy as np
import tf
import tf2_ros
from std_msgs.msg import String, Int16MultiArray
from cv_bridge import CvBridge
from duckietown_msgs.msg import BoolStamped
from fleet_planning.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from fleet_planning.srv import *
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator
from fleet_planning.transformation import Transformer
from fleet_planning.location_to_graph_mapping import IntersectionMapper

class TaxiState(Enum):
    GOING_TO_CUSTOMER = 0
    WITH_CUSTOMER = 1
    IDLE = 2


class Instruction(Enum):
    LEFT = 'l'
    RIGHT = 'r'
    STRAIGHT = 's'


class FleetPlanningStrategy(Enum): # for future expansion
    CLOSEST_DUCKIEBOT = 0


class Duckiebot:
    """tracks state and mission of one duckiebot, handles the global customer and location assignments"""

    _name = None

    _taxi_state = TaxiState.IDLE
    _last_known_location = None # number of the node the localization lastly reported
    _last_time_seen_alive = None # timestamp. updated every time a location or similar was reported. Duckiebot is removed from map if this becomes too far away from now
    _last_instruction = None  # e.g. Instruction.LEFT

    _target_location = None
    _customer_request = None # instance of CustomerRequest, only not None if on duty

    def __init__(self, robot_name, map_graph):
        self._name = robot_name
        self._map_graph = map_graph

    @property
    def taxi_state(self):
        return self._taxi_state

    @property
    def name(self):
        return self._name

    def update_location_check_target_reached(self, node_number):
        """
        updates member _last_known_location. If duckiebot is now at target location and has a customer request,
        it checks whether current location is customer start location or customer target location.
        It updates its _taxi_state correspondingly and updates sets _last_time_seen_alive and CustomerRequest time stamps.
        :param node_number: reported from localization
        :return: None if status has not changed. Returns self_taxi state if customer has been
                picked up or customer target location has been reached.
        """
        if node_number is None:
            return None

        self._last_known_location = node_number
        self._last_time_seen_alive = rospy.get_time()
        # TODO: update last instruction

        if self._customer_request is not None:
            if node_number == self._customer_request.start_location:
                self._taxi_state = TaxiState.WITH_CUSTOMER
                self._customer_request.time_pickup = rospy.get_time()
                return self._taxi_state

            elif node_number == self._customer_request.target_location:
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

    @property
    def next_location(self):
        """
        takes _last_known_location, and combines it with _last_instruction to predict where a duckiebot is going
        to be next. necessary for customer assignment search.
        :return: node number of expected next duckiebot location
        """
        # unclear where to get _last_instruction from. Since the GUI does the path planning as well,
        # maybe get it from there?
        pass

    def assign_customer_request(self, customer_request):
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

    start_location = None # node number
    target_location = None # node number

    # for the metrics. Use ropy.time() to set timestamp
    time_registered = None
    time_pickup = None
    time_drop_off = None

    def __init__(self, start_node, target_node):
        self.start_location = start_node
        self.target_location = target_node

        self.time_registered = rospy.get_time()


class TaxiCentralNode:
    TIME_OUT_CRITERIUM = 60.0

    _fleet_planning_strategy = FleetPlanningStrategy.CLOSEST_DUCKIEBOT # for now there is just this. gives room for future expansions

    _registered_duckiebots = {} # dict of instances of class Duckiebot. populated by register_duckiebot(). duckiebot name is key
    _pending_customer_requests = []
    _fulfilled_customer_requests = [] # for analysis purposes

    _map_drawing = mapDraw() # class that handles map drawing. generate_duckietown_map.py ???
    _map_graph = None # TODO: necessary ?
    _graph_creator = None

    _world_frame = 'world'
    _target_frame = 'duckiebot'

    def __init__(self, map_dir, map_csv):
        """
        subscribe to location", customer_requests. Publish to transportation status, target location.
        Init time_out timer.
        Specification see intermediate report document
        """

        gc = graph_creator()
        self._graph = gc.build_graph_from_csv(map_dir, map_csv)
        self._graph_creator = gc

        # location listener
        self._listener_transform = tf.TransformListener()
        # wait for listener setup to complete
        try:
            self._listener_transform.waitForTransform(self._world_frame,self._target_frame, rospy.Time(), rospy.Duration(4.0))
        except tf2_ros.TransformException:
            rospy.logwarn('The duckiebot location is not being published! No location updates possible.')

        # subscribers
        self._sub_customer_requests = rospy.Subscriber('~customer_requests', Int16MultiArray, self._register_customer_request, queue_size=1)
        self._sub_intersection = rospy.Subscriber('~/jeff/stop_line_filter_node/at_stop_line', BoolStamped, self._location_update)
        # publishers
        self._pub_duckiebot_target_location = rospy.Publisher('~target_location', String, queue_size=1)
        self._pub_duckiebot_transportation_status = rospy.Publisher('~transportation_status', String, queue_size=1, latch=True)
        # timers
        self._time_out_timer = rospy.Timer(rospy.Duration.from_sec(self.TIME_OUT_CRITERIUM), self._check_time_out)

        # mapping: location -> node number
        self._location_to_node_mapper = IntersectionMapper(self._graph_creator)

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
        duckiebot = Duckiebot(robot_name, self._map_graph)
        if robot_name not in self._registered_duckiebots:
            self._registered_duckiebots[robot_name] = duckiebot

        else:
            rospy.logwarn('Failed to register new duckiebot. A duckiebot with the same name has already been registered.')

    def _unregister_duckiebot(self, duckiebot):
        """unregister given duckiebot, remove from map drawing. If it currently has been assigned a customer,
        put customer request back to _pending_customer_requests"""

        request = duckiebot.pop_customer_request
        if request is not None:
            self._pending_customer_requests.append(duckiebot.pop_customer_request())

        try:
            del self._registered_duckiebots[duckiebot.name]
            rospy.logwarn('Unregistered and removed from map Duckiebot {}'.format(duckiebot.name))
        except KeyError:
            rospy.logwarn('Failure when unregistering duckiebot. {} had already been unregistered.'.format(duckiebot.name))
        # TODO: tell map to remove icon

    def _register_customer_request(self, request_msg):
        """callback function for request subscriber. appends CustomerRequest instance to _pending_customer_requests,
        Calls handle_customer_requests

        """
        start = request_msg.data[0]
        target = request_msg.data[1]
        request = CustomerRequest(start, target)
        self._pending_customer_requests.append(request)

        self._handle_customer_requests() # TODO: or better call this timer based, to assign customers in batches?

    def _handle_customer_requests(self):
        """
        Switch function. This allows to switch between strategies in the future
        """

        if self._fleet_planning_strategy == FleetPlanningStrategy.CLOSEST_DUCKIEBOT:
            self._fleet_planning_closest_duckiebot()
        else:
            raise NotImplementedError('Chosen strategy has not yet been implemented.')

    def _fleet_planning_closest_duckiebot(self):
        """
        E.g. for every pending customer request do breadth first search to find closest idle duckiebot.
        Make sure to use Duckiebot.next_location for the search. Finally assign customer request to best duckiebot.
        (Maybe if # pending_customer requests > number idle duckiebots, assign the ones with the shortest path.)
        For every assigned duckiebot, publish to target location. Publish transportation status.
        """

        # For now quickly find the closest duckiebot
        for pending_request in self._pending_customer_requests:
            idle_duckiebots = self._idle_duckiebots()
            if len(idle_duckiebots) == 0:
                rospy.loginfo("No duckiebot available for pending transport request")
                return

            # Get the start node
            start_node = self._graph.get_node(pending_request.start_location)

            nodes_to_visit = [start_node]

            # Find the closest duckiebot via breadth first search
            duckiebot = None
            while len(nodes_to_visit) > 0 and duckiebot is None:
                current_node = nodes_to_visit.pop(0)

                # Check if there's a duckiebot on that node
                for db in idle_duckiebots:
                    if str(db.name) == current_node.name:
                        # We found one!
                        duckiebot = db
                        break

                # Add all the neighboring nodes to the list of nodes we still have to visit
                edges = self._graph.node_edges(current_node)
                for edge in edges:
                    nodes_to_visit.append(edge.target)

            if duckiebot is None:
                rospy.logwarn("There are IDLE duckiebots but they were not found in the graph")

            # Assign the request to that duckiebot
            duckiebot.assign_customer_request(pending_request)

    def _location_update(self, at_stop_line):
        """
        Callback function for location subscriber. Message contains location and robot name.  If duckiebot
        is not yet known, register it first. Location is first mapped from 2d coordinates to graph node, then call
        Duckiebot.update_location_check_target_reached(..). According to its feedback move customer request to
        _fulfilled_customer_requests. If taxi has become free, call handle_customer_requests
        Update map drawing correspondingly (taxi location, customer location). Publish duckiebot taxi state
         if it has changed.
        :param location_msg: contains location and robot name
        """

        duckiebot_name = 'jeff' # TODO get this from message!!!!

        node = None
        # how to make sure we get the tf of the right duckiebot???
        start_time = rospy.get_time()
        # TODO: this whole loop here is done locally
        while not node and rospy.get_time() - start_time < 5.0: # TODO: tune this
            try:
                (trans, rot) =self._listener_transform.lookupTransform(self._world_frame, self._target_frame, rospy.Time(0))

                if trans[2] != 1000: # the localization package uses this to encode that no information about the location exists. (here == 1 km in the air)
                    rot = tf.transformations.euler_from_quaternion(rot)[2]
                    node = self._location_to_node_mapper.get_node_name(trans[:2], np.degrees(rot))
                    rospy.logwarn("Current location ({},{}) corresponds to node {}.".format(trans[0], trans[1], node))

            except tf2_ros.LookupException:
                rospy.logwarn('Duckiebot: {} location transform not found. Trying again.'.format(duckiebot_name))

        if not node:
            rospy.logwarn('Duckiebot: {} location update failed. Location not updated.'.format(duckiebot_name))
            return

        if duckiebot_name not in self._registered_duckiebots:
            self._create_and_register_duckiebot(duckiebot_name)
            duckiebot = self._registered_duckiebots[duckiebot_name]
            new_duckiebot_state = duckiebot.update_location_check_target_reached(node)

        else:
            duckiebot = self._registered_duckiebots[duckiebot_name]
            new_duckiebot_state = duckiebot.update_location_check_target_reached(node)

        if new_duckiebot_state == TaxiState.IDLE: # mission accomplished
            request = duckiebot.pop_customer_request()
            self._fulfilled_customer_requests.append(request)
            self._handle_customer_requests() # bcs duckiebot is available again
            self._pub_duckiebot_transportation_status(duckiebot)
            # TODO: remove customer icon from map

        elif new_duckiebot_state == TaxiState.WITH_CUSTOMER: # reached customer
            self._publish_duckiebot_transportation_status(duckiebot)
            # TODO: make customer icon move with duckiebot icon

        else: # nothing special happened, just location update
            pass

        # TODO draw duckiebot location

    def _check_time_out(self, msg):
        """callback function from some timer, ie. every 30 seconds. Checks for every duckiebot whether it has been
        seen since the last check_time_out call. If not, unregister duckiebot"""

        for duckiebot in self._registered_duckiebots.values():
            if duckiebot.has_timed_out(self.TIME_OUT_CRITERIUM):
                rospy.logwarn('Duckiebot {} has timed out.'.format(duckiebot.name))
                self._unregister_duckiebot(duckiebot)

    def _publish_duckiebot_transportation_status(self, duckiebot):
        """ is called whenever the taxi_state of a duckiebot changes, publish this information to
        transportatkion status topic"""
        self._pub_duckiebot_transportation_status(duckiebot.name + 'status' + str(duckiebot.taxi_state))

    def save_metrics(self): # implementation has rather low priority
        """ gather timestamps from customer requests, calculate metrics, save to json file"""
        pass

    def on_shutdown(self):
        rospy.loginfo("[TaxiCentralNode] Shutdown.")


class graph_search_server():
    def __init__(self):
        print 'Graph Search Service Started'

        # Loading paths
        self.script_dir = os.path.dirname(__file__)
        self.map_path = self.script_dir + '/maps/' + self.map_name
        self.map_img_path = self.map_path + '_map'
        self.tiles_dir = os.path.abspath(
            self.script_dir + '../../../../30-localization-and-planning/duckietown_description/urdf/meshes/tiles/')

        # build and init graphs
        gc = graph_creator()
        self.duckietown_graph = gc.build_graph_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)
    
        print "Map loaded successfully!\n"

        map_draw = mapDraw(self.duckietown_graph, self.duckietown_problem)

    def handle_graph_search(self, req):
        """takes request, calculates path and creates corresponding graph image. returns path"""
        # Checking if nodes exists
        if (req.source_node not in self.duckietown_graph) or (req.target_node not in self.duckietown_graph):
            print "Source or target node do not exist."
            self.publishImage(req, [])
            return GraphSearchResponse([])

        # Running A*
        self.duckietown_problem.start = req.source_node
        self.duckietown_problem.goal = req.target_node
        path = self.duckietown_problem.astar_search()

        # Publish graph solution
        self.publishImage(req, path)

        return GraphSearchResponse(path.actions)        
    



class mapDraw():
    """
    Used to generate the map from a csv file, draw the graph on top 
    of that and draw the icons for each duckiebot.
    """
    def __init__(self, duckietown_graph, duckietown_problem):
        print 'mapDraw initializing...'

        # Input: csv file
        self.map_name = rospy.get_param('/map_name')

        # Loading paths
        self.script_dir = os.path.dirname(__file__)
        self.map_path = self.script_dir + '/maps/' + self.map_name
        self.map_img_path = self.map_path + '_map'
        self.tiles_dir = os.path.abspath(
            self.script_dir + '../../../../30-localization-and-planning/duckietown_description/urdf/meshes/tiles/')
        self.customer_icon_path = os.path.abspath(self.script_dir + '/../include/gui_images/customer_duckie.jpg')
        self.start_icon_path = os.path.abspath(self.script_dir + '/../include/gui_images/duckie.jpg')
        self.target_icon_path = os.path.abspath(self.script_dir + '/../include/gui_images/location-icon.png')

        # build and init graphs
        #gc = graph_creator()
        self.duckietown_graph = duckietown_graph #gc.build_graph_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)
        self.duckietown_problem = duckietown_problem # GraphSearchProblem(self.duckietown_graph, None, None)
    
        print "Map loaded successfully!\n"

        self.image_pub = rospy.Publisher("~map_graph",Image, queue_size = 1, latch=True)
        self.bridge = CvBridge()

        # prepare and send graph image through publisher
        self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name = self.map_name)

        mc = MapImageCreator(self.tiles_dir)
        self.tile_length = mc.tile_length
        self.map_img = mc.build_map_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)

        # image used to store all start, customer and target icons at their positions
        print(self.customer_icon_path)
        self.customer_icon = cv2.resize(cv2.imread(self.customer_icon_path), (30, 30))
        self.start_icon = cv2.resize(cv2.imread(self.start_icon_path), (30, 30))
        self.target_icon = cv2.resize(cv2.imread(self.target_icon_path), (30, 30))
        
        overlay = self.prepImage()
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))
    
    def graph_node_to_image_location(self, graph, node):
        """
        Convert a graph node number to a 2d image pixel location
        """
        print "graph.node_positions", graph.node_positions
        return graph.node_positions[node]

    def draw_icons(self, map_image, icon_type, location ):
        """
        Draw start, customer and target icons next to each 
        corresponding graph node along with the respective name 
        of the duckiebot. 
        Input:
            - map_image: the base map image onto which to draw the icons
            - icon_type: string, either customer, start or target
            - location: where to draw the icon, as a graph node number
        
        Returns:
            - opencv image with the icons at the correct positions
        """
        #print "Size of map: ", self.map_image.shape

        # loop through all trips currently in existence. For each trip,
        # draw the start, customer and target icons next to the corresponding 
        # label of the graph node. 
        print "self.map_img.shape: ", self.map_img.shape
        transf = Transformer(self.tile_length, self.map_img.shape[0] / self.tile_length)  # TODO: better way to get the map dimensions?
        if icon_type == "customer":
            icon = self.customer_icon
        elif icon_type == "start":
            icon = self.start_icon
        elif icon_type == "target":
            icon = self.target_icon
        else: 
            print "invalid icon type"
            # return

        # convert graph number to 2D image pixel coords
        point = self.graph_node_to_image_location(graph = self.duckietown_graph, node = location)
        print "Point received is: ", point
        point = transf.map_to_image(point)
        print "Point received is: ", point
        x_start = point[1]
        x_end = x_start + icon.shape[0]
        y_start = point[0]  
        y_end = y_start + icon.shape[1]
        map_image[x_start:x_end, y_start:y_end, :] = icon

        return map_image

    def publishImage(self, req, path):
        if path:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=path.edges(), map_name=self.map_name,
                                       highlight_nodes=[req.source_node, req.target_node])
        else:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name=self.map_name)

        print req.source_node, req.target_node
        # TODO: either pass the name of the nodes here and use the  self.duckietown_graph.get_node_oos(node) function
        # or figure out some other way to get the location. 
        overlay = self.prepImage()
        print "req: ", req.source_node, req.target_node
        # draw request if initialized, i.e. nonzero
        if req.target_node != '0':
            overlay = self.draw_icons(overlay, "start", location = req.source_node)
            overlay = self.draw_icons(overlay, "target", location = req.target_node) 
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

    def prepImage(self):
        """takes the graph image and map image and overlays them"""
        # TODO: add the icon image and merge it as well
        inverted_graph_img = 255 - self.graph_image
        # bring to same size
        inverted_graph_img = cv2.resize(inverted_graph_img, (self.map_img.shape[1], self.map_img.shape[0]))

        # overlay images
        overlay = cv2.addWeighted(inverted_graph_img, 1, self.map_img, 0.5, 0)

        # make the image bright enough for display again
        hsv = cv2.cvtColor(overlay, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        lim = 255 - 60
        v[v > lim] = 255
        v[v <= lim] += 60
        final_hsv = cv2.merge((h, s, v))

        overlay = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return overlay

if __name__ == '__main__':
    # startup node
    rospy.init_node('taxi_central_node')

    script_dir = os.path.dirname(__file__)
    map_path = os.path.abspath(script_dir)
    csv_filename = 'tiles_lab'

    taxi_central_node = TaxiCentralNode(map_path, csv_filename)
    
    gss = graph_search_server()
    print 'Starting server...\n'
    s = rospy.Service('graph_search', GraphSearch, gss.handle_graph_search)    

    rospy.on_shutdown(taxi_central_node.on_shutdown)
    rospy.spin()
