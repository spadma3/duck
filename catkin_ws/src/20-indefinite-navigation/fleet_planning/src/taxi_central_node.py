from enum import Enum


class TaxiState(Enum):
    GOING_TO_CUSTOMER = 0
    WITH_CUSTOMER = 1
    IDLE = 2


class Insctruction(Enum):
    LEFT = 'l'
    RIGHT = 'r'
    STRAIGHT = 's'


class FleetPlanningStrategy(Enum): # for future expansion
    CLOSEST_DUCKIEBOT = 0


class Duckiebot:
    """tracks state and mission of every duckiebot, handles the global customer and location assignments"""
    _duckiebot_name = None

    _taxi_state = TaxiState.IDLE
    _last_known_location = None # number of the node the localization lastly reported
    _last_time_seen_alive = None # timestamp. updated every time a location or similar was reported. Duckiebot is removed from map if this becomes too far away from now
    _last_instruction = None  # e.g. Instruction.LEFT

    _target_location = None
    _customer_request = None # instance of CustomerRequest, only not None if on duty

    _map_graph = None # reference to graph

    def __init__(self, robot_name, map_graph):
        self._duckiebot_name = robot_name
        self._map_graph = map_graph

    def update_location_check_target_reached(self, node_number):
        """
        updates member _last_known_location. If duckiebot is now at target location and has a customer request,
        it checks whether current location is customer start location or customer target location.
        It updates its _taxi_state correspondingly and updates sets _last_time_seen_alive and CustomerRequest time stamps.
        :param node_number: reported from localization
        :return: None if status has not changed. Returns self_taxi state if customer has been
                picked up or customer target location has been reached.
        """

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

        
class CustomerRequest:
    _start_location = None # node number
    _target_location = None # node number

    # for the metrics. Use time.time() to set timestamp
    _time_registered = None
    _time_pickup = None
    _time_dropoff = None


class TaxiCentralNode:
    _fleet_planning_strategy = FleetPlanningStrategy.CLOSEST_DUCKIEBOT # for now there is just this. gives room for future expansions

    _registered_duckiebots = [] # list of instances of class Duckiebot. populated by register_duckiebot()
    _pending_customer_requests = []
    _fulfilled_customer_requests = [] # for analysis purposes

    _map_drawing = None # class that handles map drawing. generate_duckietown_map.py ???
    _map_graph = None

    def __init__(self, map_drawing, map_graph):
        """
        subscribe to location", transportation_requests. Publish to transportation status, target location.
        Init time_out timer.
        Specification see intermediate report document
        :param map_drawing: class that handles the drawing of the map
        :param map_graph: graph structure to do the duckiebot assignment graph search
        """
        pass

    def register_duckiebot(self, robot_name):
        """
        Whenever a new duckiebot is detected, this method is called. Create Duckiebot instance and append to _registered_duckiebots
        E.g. an unknown duckiebot publishes a location -> register duckiebot
        :param Duckiebot:
        """
        pass

    def unregister_duckiebot(self, duckiebot):
        """unregister given duckiebot, remove from map drawing. If it currently has been assigned a customer,
        put customer request back to _pending_customer_requests"""
        pass

    def register_customer_request(self, start_location, target_location):
        """callback function for request subscriber. appends CustomerRequest instance to _customer_requests,
        sets time stamp _time_registered. Calls handle_customer_requests

        """
        pass

    def handle_customer_requests(self):
        """
        Switch function. This allows to switch between strategies in the future
        """

        if self._fleet_planning_strategy == FleetPlanningStrategy.CLOSEST_DUCKIEBOT:
            self.fleet_planning_closest_duckiebot()
        else:
            raise NotImplementedError('Chosen strategy has not yet been implemented.')

    def fleet_planning_closest_duckiebot(self):
        """
        E.g. for every pending customer request do breadth first search to find closest idle duckiebot.
        Make sure to use Duckiebot.next_location for the search. Finally assign customer request to best duckiebot.
        (Maybe if # pending_customer requests > number idle duckiebots, assign the ones with the shortest path.)
        For every assigned duckiebot, publish to target location.
        """

    def location_update(self, location_msg):
        """
        Callback function for location subscriber. Message contains location and robot name.  If duckiebot
        is not yet known, register it first. Location is first mapped from 2d coordinates to graph node, then call
        Duckiebot.update_location_check_target_reached(..). According to its feedback move customer request to
        _fulfilled_customer_requests. If taxi has become free, call handle_customer_requests
        Update map drawing correspondingly (taxi location, customer location). Publish duckiebot taxi state
         if it has changed.
        :param location_msg: contains location and robot name
        """

        # call corresponding Duckiebot.update_location_check_target_reached(..)
        # update map drawing
        pass

    def check_time_out(self):
        """callback function from some timer, ie. every 30 seconds. Checks for every duckiebot whether it has been
        seen since the last check_time_out call. If not, deregister duckiebot"""
        pass

    def publish_duckiebot_transportation_status(self, taxi_state):
        """ is called whenever the taxi_state of a duckiebot changes, publish this information to
        transportatkion status topic"""
        pass
