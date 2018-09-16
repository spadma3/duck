#!/usr/bin/env python
import rospy
from std_msgs.msg import ByteMultiArray
from fleet_planning.srv import GraphSearch, VirtualDuckiebotLocation, VirtualDuckiebot
from fleet_planning.duckiebot import BaseDuckiebot, TaxiState
from message_serialization import *
from collections import namedtuple


class DuckiebotMission:
    def __init__(self, name, location, path=[]):
        self.name = name
        self.location = location
        self.path = path


class VirtualDuckiebotNode:
    def __init__(self):
        # Listen to commands
        self._sub_command_topic = rospy.Subscriber(
            "/taxi/commands",
            ByteMultiArray,
            self._received_taxi_command
        )

        # listen to location updates

        # Publish localization updates
        self._pub_localization_topic = rospy.Publisher(
            "/taxi/location",
            ByteMultiArray,
            queue_size=1
        )

        self._duckiebots = {}
        self._timer_location_update = rospy.Timer(rospy.Duration.from_sec(2.0), self.update_locations)

        rospy.loginfo("Finished Virtual Duckiebot Node startup")

    def onShutdown(self):
        rospy.loginfo("Shutting down virtual duckiebot")

    def create_duckiebot(self, request):
        """publish duckiebot location update at given location"""
        location = request.location
        name = request.name
        self._duckiebots[name] = DuckiebotMission(name, str(location), [])
        rospy.wait_for_service('send_location_information')
        send_location = rospy.ServiceProxy('send_location_information', VirtualDuckiebotLocation)
        resp = send_location(name, location, '')
        return 'success'

    def remove_duckiebot(self, request):

        if request.name not in self._duckiebots.iterkeys():
            return "no such duckiebot"

        del self._duckiebots[request.name]
        return 'success'

    def update_locations(self, msg):
        rospy.loginfo('Go one timestep ahead. Update locations')
        for db in self._duckiebots.itervalues():
            if db.path is None:
                continue

            db_path = [p for p in db.path if p.isdigit() and int(p)%2 == 1]
            for i, p in enumerate(db_path):
                if p == db.location:

                    if p == db_path[-1]:
                        db.path = None
                    else:
                        db.location = db_path[i+1]
                        self.update_location(db)
                        break

    def update_location(self, duckiebot):
        rospy.wait_for_service('send_location_information', timeout=1.0)
        send_location = rospy.ServiceProxy('send_location_information', VirtualDuckiebotLocation)
        resp = send_location(duckiebot.name, int(duckiebot.location), ','.join(duckiebot.path))
        self._duckiebots.update({duckiebot.name: duckiebot})

    def send_location_information(self, req):
        rospy.loginfo("Will send location information")

        # Serialize Message and send it
        route = req.route.split(',')
        serialized_message = LocalizationMessageSerializer.serialize(req.duckie_name, req.location, route)
        message = ByteMultiArray(data=serialized_message)

        self._pub_localization_topic.publish(message)

        return "success"

    def _received_taxi_command(self, mes):
        rospy.loginfo("Received taxi command")

        # Deserialize message and print it
        name, target_node, command = InstructionMessageSerializer.deserialize("".join(map(chr, mes.data)))
        rospy.loginfo("Duckie Name: %s", name)
        rospy.loginfo("Target Node: %d", target_node)
        rospy.loginfo("Command: %d", command)

        source_node = self._duckiebots[name].location
        # calculate path
        rospy.wait_for_service('/laptop/graph_search', timeout=1.0)

        graph_search = rospy.ServiceProxy('/laptop/graph_search', GraphSearch)
        resp = graph_search(str(source_node), str(target_node))
        self._duckiebots[name].path = resp.path
        self.update_location(self._duckiebots[name])


if __name__ == "__main__":
    rospy.init_node('virtual_duckiebot_node')
    virtual_duckiebot_node = VirtualDuckiebotNode()

    send_location_service = rospy.Service(
        'send_location_information',
        VirtualDuckiebotLocation,
        virtual_duckiebot_node.send_location_information
    )

    create_duckiebot_service = rospy.Service(
        'create_duckiebot',
        VirtualDuckiebot,
        virtual_duckiebot_node.create_duckiebot
    )

    remove_duckiebot_service = rospy.Service(
        'remove_duckiebot',
        VirtualDuckiebot,
        virtual_duckiebot_node.remove_duckiebot
    )

    rospy.on_shutdown(virtual_duckiebot_node.onShutdown)
    rospy.spin()
