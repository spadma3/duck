import rospy
from std_msgs.msg import ByteMultiArray
from fleet_planning.srv import *

class VirtualDuckiebotNode:
    def __init__(self):
        # Listen to commands
        self._sub_command_topic = rospy.Subscriber(
            "/taxi/commands",
            ByteMultiArray,
            self._received_taxi_command
        )

        # Publish localization updates
        self._pub_localization_topic = rospy.Publisher(
            "/taxi/location",
            ByteMultiArray,
            queue_size=1
        )
        rospy.loginfo("Finished Virtual Duckiebot Node startup")

    def onShutdown(self):
        rospy.loginfo("Shutting down virtual duckiebot")

    def send_location_information(self, req):
        rospy.loginfo("Will send location information")

        # TODO: Serialize Message and send it

        return "success"

    def _received_taxi_command(self, data):
        rospy.loginfo("Received taxi command")
        # TODO: Deserialize message and print it


if __name__ == "__main__":
    rospy.init_node('virtual_duckiebot_node')
    virtual_duckiebot_node = VirtualDuckiebotNode()

    send_location_service = rospy.Service(
        'send_location_information',
        VirtualDuckiebotLocation,
        virtual_duckiebot_node.send_location_information
    )

    rospy.on_shutdown(virtual_duckiebot_node.onShutdown)
    rospy.spin()
