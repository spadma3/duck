#!/usr/bin/env python

import sys
import rospy
from taxi_central_node import CustomerRequest, Duckiebot, TaxiCentralNode, TaxiState
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
        assert(duckiebot.get_taxi_state() == TaxiState.IDLE)

        duckiebot.assign_customer_request(request)
        assert(duckiebot._customer_request == request)
        assert(duckiebot.get_taxi_state() == TaxiState.GOING_TO_CUSTOMER)

        assert(duckiebot.update_location_check_target_reached(start) == TaxiState.WITH_CUSTOMER)
        assert (duckiebot.get_taxi_state() == TaxiState.WITH_CUSTOMER)

        assert (duckiebot.update_location_check_target_reached(other) is None)
        assert (duckiebot.get_taxi_state() == TaxiState.WITH_CUSTOMER)

        assert (duckiebot.update_location_check_target_reached(stop) == TaxiState.IDLE)
        assert (duckiebot.get_taxi_state() == TaxiState.IDLE)

        assert(duckiebot.pop_customer_request() == request)
        assert(duckiebot._customer_request is None)




if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_taxi_central', TestTaxiCentral)