import rospy
from std_msgs.msg import ByteMultiArray

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
            ByteMultiArray
        )

    def _received_taxi_command(self, data):
        rospy.loginfo("Received taxi command")
        pass


if __name__ == "__main__":
    rospy.init_node('virtual_duckiebot_node')
    virtual_duckiebot_node = VirtualDuckiebotNode()
    rospy.on_shutdown(virtual_duckiebot_node.onShutdown)
    rospy.spin()
