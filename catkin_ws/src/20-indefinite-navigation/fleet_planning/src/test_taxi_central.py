#!/usr/bin/env python

import sys
import rospy
import os
from taxi_central_node import CustomerRequest, Duckiebot, TaxiCentralNode, TaxiState
from std_msgs.msg import Int16MultiArray
from duckietown_msgs.msg import BoolStamped
from fleet_planning.generate_duckietown_map import graph_creator
PKG = 'fleet_planning'

import unittest

class TestTaxiCentral(unittest.TestCase):

    def test_duckiebot_location_update(self):
        rospy.init_node('test_node')

        duckiebot = Duckiebot('paco', None)
        start = 5
        stop = 12
        other = 7
        request = CustomerRequest(start, stop)

        # nothing to do, idle
        self.assertEqual(duckiebot.taxi_state, TaxiState.IDLE)

        # new request!
        duckiebot.assign_customer_request(request)

        self.assertEqual(duckiebot._customer_request, request)
        self.assertEqual(duckiebot.taxi_state, TaxiState.GOING_TO_CUSTOMER)

        # now duckiebot is at customer start location
        self.assertEqual(duckiebot.update_location_check_target_reached(start), TaxiState.WITH_CUSTOMER)
        self.assertEqual(duckiebot.taxi_state, TaxiState.WITH_CUSTOMER)

        # now duckiebot is at any other location
        self.assertEqual(duckiebot.update_location_check_target_reached(other), None)
        self.assertEqual(duckiebot.taxi_state, TaxiState.WITH_CUSTOMER)

        # now duckiebot is at customer target location
        self.assertEqual(duckiebot.update_location_check_target_reached(stop), TaxiState.IDLE)
        self.assertEqual(duckiebot.taxi_state, TaxiState.IDLE)

        # customer request removed
        self.assertEqual(duckiebot.pop_customer_request(), request)
        self.assertEqual(duckiebot._customer_request, None)

    def test_taxi_central_node(self):
        # startup node
        script_dir = os.path.dirname(__file__)
        map_path = os.path.abspath(script_dir)
        csv_filename = 'tiles_lab'

        gc = graph_creator()
        gc.build_graph_from_csv(map_path, csv_filename)
        taxi_central_node = TaxiCentralNode(gc)

        request = Int16MultiArray()
        request.data=[5, 17]

        # register customer request
        taxi_central_node._register_customer_request(request)
        self.assertTrue(isinstance(taxi_central_node._pending_customer_requests[0], CustomerRequest))

        # location update handling
        robot_name = 'paco'
        at_stop_line = BoolStamped()
        at_stop_line.data = True
        taxi_central_node._location_update(at_stop_line)
        # no locations are published. Thus paco shall not be registered, since unknown location.
        self.assertFalse(robot_name in taxi_central_node._registered_duckiebots)

        # make new duckiebot, assign a customer request to it
        taxi_central_node._create_and_register_duckiebot(robot_name)
        taxi_central_node._registered_duckiebots[robot_name].update_location_check_target_reached(5)
        taxi_central_node._register_customer_request(request)
        request_new = taxi_central_node._pending_customer_requests.pop()
        taxi_central_node._registered_duckiebots[robot_name]._customer_request = request_new

        self.assertTrue(robot_name in taxi_central_node._registered_duckiebots)

        # set timer to shorter period, test robot time out deregistration
        taxi_central_node.TIME_OUT_CRITERIUM = 1.0
        self._time_out_timer = rospy.Timer(rospy.Duration.from_sec(taxi_central_node.TIME_OUT_CRITERIUM),
                                           taxi_central_node._check_time_out)
        rospy.sleep(1.2)
        # removed bcs of time out
        self.assertTrue(robot_name not in taxi_central_node._registered_duckiebots)

        # request shall be pending again
        rospy.logwarn(taxi_central_node._pending_customer_requests)

        rospy.logwarn(request_new)
        self.assertTrue(request_new in taxi_central_node._pending_customer_requests)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_taxi_central', TestTaxiCentral)
