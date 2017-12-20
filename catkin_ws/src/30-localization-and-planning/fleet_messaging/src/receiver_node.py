#!/usr/bin/env python


# Imports
import thread
import rospy
import fleet_messaging.commlibs2 as cl
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import MultiArrayDimension
from ruamel.yaml  import YAML


class Receiver(object):
    """
    Receiver class for the duckietown fleet messaging.
    Listens to an message and then sends out the respective topic to other ROS
    nodes.
    """
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Load the parameters
        config_path = self.setup_parameter("~config")
        self.iface = self.setup_parameter("~iface")

        # Load the configuration
        try:
            with open(config_path, "r") as config_file:
                # Load the configuration file
                yaml_obj = YAML()
                config_yaml = yaml_obj.load(config_file)
        except IOError:
            output = "[%s] File \"%s\" does not exist! Please use an " + \
                     "existing file!"
            rospy.logfatal(output %(self.node_name, config_path))
            raise

        # Loop through the configuration
        try:
            self.config = {}
            for entry in config_yaml:
                # Create the socket
                port = entry["port"]
                socket = cl.DuckieMQ(self.iface, port, "sub")

                # Create the publisher
                pub_topic = entry["pub"]
                pub = rospy.Publisher(pub_topic, ByteMultiArray, queue_size=1)

                # Populate the configuration
                self.config[entry["name"]] = (
                    port,
                    pub_topic,
                    pub,
                    socket
                )
        except TypeError:
            output = "[%s] Syntax error in \"%s\"!"
            rospy.logfatal(output %(self.node_name, config_path))
            raise

        # &FEF - Create and start the threads

    def setup_parameter(self, param_name, default_value=None):
        """
        Setup a node parameter.
        Inputs:
        - param_name:    Parameter name (see launch file)
        - default_value: Default parameter value (define in launch file!)
        Outputs:
        - value: Parameter value
        """
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)

        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name, value))

        return value

    def on_shutdown(self):
        """
        Perform cleanup on shutdown.
        """
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

        # Loop through the configuration
        for key in self.config:
            # &FEF - Stop the threads

            # Destroy the sockets
            self.config[key][3].cleanup()


def publish_msg(socket, pub, evnt, timeout):
    """
    Receive a ZeroMQ message and publish it as a ROS topic.
    This function is going to be called in a thread.
    Inputs:
    - socket:  ZeroMQ socket
    - pub:     ROS publisher
    - evnt:    Event
    - timeout: Timeout
    Outputs:
    None
    """
    # Loop until the event fires
    while not evnt.isSet():
        # If the event fires, exit the function
        event_is_set = evnt.wait(timeout)
        if event_is_set:
            return

        # Receive, process and publish the message
        zmq_msg = socket.rcv_serialized()
        bma = ByteMultiArray()
        bma.layout.data_offset = zmq_msg.layout.data_offset
        for dim in zmq_msg.layout.dim:
            mad = MultiArrayDimension()
            mad.label = dim.label
            mad.size = dim.size
            mad.stride = dim.stride
            bma.layout.dim.append(mad)
        bma.data = zmq_msg.data
        pub.publish(bma)


if __name__ == "__main__":
    # Initialize the node with rospy
    rospy.init_node('receiver_node', anonymous=False)

    # Create the sender object
    RECEIVER = Receiver()

    # Setup proper shutdown behavior
    rospy.on_shutdown(RECEIVER.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
