from enum import Enum, IntEnum
import rospy
from fleet_planning.duckiebot import *
import json

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


class BaseCustomerRequest:
    def __init__(self, start_node, target_node):
        self.start_location = start_node  # node number
        self.target_location = target_node  # node number

    @classmethod
    def from_json(cls, json_data):
        if json_data == '':
            return None

        start_location = json_data['start_location']
        target_location = json_data['target_location']
        return cls(start_location, target_location)


class CustomerRequest(BaseCustomerRequest):
    def __init__(self, start_node, target_node):
        self.start_location = start_node  # node number
        self.target_location = target_node  # node number

        # for the metrics. Use ropy.time() to set timestamp
        self.time_registered = rospy.get_time()
        self.time_pickup = None
        self.time_drop_off = None

    def to_dict(self):
        dict = {'target_location': self.target_locattion, 'start_location': self.start_location}
        return dict

class BaseDuckiebot:
    def __init__(self, name, taxi_state, location, next_locatoin, customer_request):
        self._name = name
        self._taxi_state = taxi_state

        self._last_known_location = location  # number of the node the localization lastly reported
        self._next_expected_location = next_locatoin
        self._customer_request = customer_request  # instance of CustomerRequest, only not None if on duty

    @classmethod
    def from_json(cls, json_data):

        name = json_data['robot_name']
        taxi_state = TaxiState(json_data['taxi_state'])
        last_known_location = json_data['location']
        next_expected_location = json_data['next_location']
        customer_request_data= json_data['customer_request']
        customer_request = CustomerRequest.from_json(customer_request_data)

        return cls(name, taxi_state, last_known_location, next_expected_location, customer_request)

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
    def location(self):  # This can be adapted to take into account some between-intersections location estimation
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


class Duckiebot(BaseDuckiebot):
    """tracks state and mission of one duckiebot, handles the global customer and location assignments"""

    def __init__(self, robot_name):
        self._name = robot_name
        self._taxi_state = TaxiState.IDLE

        self._last_known_location = None  # number of the node the localization lastly reported
        self._next_expected_location = None
        self._last_time_seen_alive = None  # timestamp. updated every time a location or similar was reported. Duckiebot is removed from map if this becomes too far in the past

        self._target_location = None
        self._customer_request = None  # instance of CustomerRequest, only not None if on duty

    def to_dict(self):
        if self._customer_request is not None:
            customer_dict = self._customer_request.to_dict()
        else:
            customer_dict = ''
        dict = {'robot_name': self._name, 'taxi_state': self._taxi_state.value, 'location': self._last_known_location,
                'next_location': self._next_expected_location, 'customer_request': customer_dict}

        return dict

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
