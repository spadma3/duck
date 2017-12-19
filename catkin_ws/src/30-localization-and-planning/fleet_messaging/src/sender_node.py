#!/usr/bin/env python
import rospy
import fleet_messaging.commlibs2 as cl
from std_msgs.msg import ByteMultiArray
from ruamel.yaml  import YAML

class Sender(object):
    """Listens to an outox topic and then sends out msg to communication network"""
    def __init__(self):
        # Initialize node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." % (self.node_name))
        # Instantiates ROS subscriber
        #self.subscriber = rospy.Subscriber("fleet_planning_outbox", ByteMultiArray, self.to_send_cb)

        # Load the parameters
        config_path = self.setup_parameter("~config", None)

        # with open("config.yaml", "r") as config_file:
        #     yaml_obj = YAML()
        #     config_yaml = yaml_obj.load(config_file)





        # Wireless Interface
        # self.iface = self.setup_parameter("~iface", "wlan0")
        # rospy.loginfo("Interface: %s" %self.iface)
        # ZMQ Publisher
        # self.out_socket = cl.Duckiemq(interface=self.iface, socktype='pub')

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def create_cb(self, msg, out_socket):

        def to_send_cb(self, msg):
            """A call back that sends out message to communication network"""
            # flag = "Fleet_Planning:"
            # mail = "%s %s" % (flag, msg.data)
            # out_socket.send_string(mail)
            # rospy.loginfo("Sending: " + mail)
            ts = rospy.Time.now()
            out_socket.send_serialized(msg.data)
            rospy.loginfo("Sending msg at time: " + str(ts))

        return to_send_cb

rospy.init_node('sender_node', anonymous=False)
sender = Sender()
rospy.spin() #Keeps the script for exiting