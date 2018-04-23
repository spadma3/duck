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

from duckietown_utils import tcp_communication
class TCPCommunicationClientNode(object):
    def __init__(self):
        self.node_name = "TCP Communication Client Node"

        ## setup Parameters
        self.setupParams()


        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)



        self.service_getVariable = rospy.Service("~get_variable", GetVariable, self.getVariable)
        self.service_setVariable = rospy.Service("~set_variable", SetVariable, self.setVariable)

        ans = tcp_communication.getTCPVariable("julien")

        rospy.loginfo("ans: " + ans)

    def getVariable(self, req):
        rospy.loginfo("YEEEEEEEEEEEEE")
        s = String()
        s.data = "aaaaa"
        return s
        MESSAGE = ["GET", json.loads(req.name)]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.IP, self.PORT))
        s.send(json.dumps(MESSAGE))
        data = s.recv(self.BUFFER_SIZE)
        response = data
        s.close()

        return response

    def setVariable(self, req):
        MESSAGE = ["SET", json.loads(req.name), json.loads(req.value)]
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.IP, self.PORT))
        s.send(json.dumps(MESSAGE))
        data = s.recv(self.BUFFER_SIZE)
        response = data
        s.close()

        return response


    def setupParams(self):
        self.IP = self.setupParam("~IP", "192.168.1.205")
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
