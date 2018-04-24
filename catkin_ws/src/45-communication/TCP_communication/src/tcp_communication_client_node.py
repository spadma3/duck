#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList
from std_msgs.msg import Float32, Int16, Bool, String
from geometry_msgs.msg import Point
import time
import math
import socket
import json
from duckietown_msgs.srv import GetVariable, SetVariable
from duckietown_utils import robot_name

from duckietown_utils import tcp_communication
class TCPCommunicationClientNode(object):
    def __init__(self):
        self.node_name = "TCP Communication Client Node"
        self.veh_name = robot_name.get_current_robot_name()
        ## setup Parameters
        self.setupParams()


        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


        # Setting up rosservices
        self.service_getVariable = rospy.Service("~get_variable", GetVariable, self.getVariable)
        self.service_setVariable = rospy.Service("~set_variable", SetVariable, self.setVariable)


    # Get variable from variable server (rosservice function)
    def getVariable(self, req):
        # Data is passed in JSON format (from rosservice and also TCP)
        # MESSAGE = [VEHICLE_NAME, ACTION, VAR_NAME(, VAR_VALUE)]
        MESSAGE = [self.veh_name, "GET", json.loads(req.name_json.data)]
        # Check if data is too long
        if len(MESSAGE) > self.BUFFER_SIZE:
            string = String()
            string.data = json.dumps("ERROR")
            return string

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.IP, self.PORT))
            s.send(json.dumps(MESSAGE))
            response = s.recv(self.BUFFER_SIZE)
            s.close()
        except:
            rospy.loginfo("[TCP Communication Client Node] Could not connect!")
            response = json.dumps("ERROR")
        # Create return String
        string = String()
        string.data = response

        return string

    # Sett variable on variable server (rosservice function)
    def setVariable(self, req):
        # Data passed in JSON format
        # MESSAGE = [VEHICLE_NAME, ACTION, VAR_NAME(, VAR_VALUE)]
        MESSAGE = [self.veh_name, "SET", json.loads(req.name_json.data), json.loads(req.value_json.data)]

        # Check if data is too long
        if len(MESSAGE) > self.BUFFER_SIZE:
            string = String()
            string.data = json.dumps("ERROR")
            return string

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.IP, self.PORT))
            s.send(json.dumps(MESSAGE))
            response = s.recv(self.BUFFER_SIZE)
            s.close()
        except:
            rospy.loginfo("[TCP Communication Client Node] Could not connect!")
            response = json.dumps("ERROR")
        # Create return String
        string = String()
        string.data = response
        return string


    def setupParams(self):
        self.IP = self.setupParam("~IP", "192.168.1.222")
        self.PORT = self.setupParam("~PORT", 5678)
        self.BUFFER_SIZE = self.setupParam("~BUFFER_SIZE", 1024)

    def updateParams(self,event):
        self.IP = rospy.get_param("~IP")
        self.PORT = rospy.get_param("~PORT")
        self.BUFFER_SIZE = rospy.get_param("~BUFFER_SIZE")


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[TCPCommunicationClientNode] Shutdown.")
        self.tcp_socket.close()

if __name__ == '__main__':
    rospy.init_node('tcp_communication_client_node',anonymous=False)
    tcp_communication_client_node = TCPCommunicationClientNode()
    rospy.on_shutdown(tcp_communication_client_node.onShutdown)
    rospy.spin()
