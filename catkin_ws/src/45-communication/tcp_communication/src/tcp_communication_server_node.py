#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point
import time
import math
import socket
import json

class TCPCommunicationServerNode(object):
    def __init__(self):
        self.node_name = "TCP Communication Server Node"

        ## setup Parameters
        self.setupParams()


        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        # Prepare socket
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.bind((self.IP, self.PORT))
        self.tcp_socket.listen(1)

        # Listen
        while not rospy.is_shutdown():
            try:
                self.mainLoop()
            except:
                rospy.loginfo("Error")
        self.tcp_socket.close()

    def mainLoop(self):
        # Wait for and accept connection
        tcp_connection, addr = self.tcp_socket.accept()
        data_raw = tcp_connection.recv(self.BUFFER_SIZE)
        
        # Decode JSON format
        data = json.loads(data_raw)

        # data = [VEHICLE_NAME, ACTION, VAR_NAME(, VAR_VALUE)]
        if data[1] == "SET":
            rospy.set_param("~" + data[2], data[3])
            response = True
            rospy.loginfo(str(data[0]) +  " sets " + str(data[2]) + " to " + str(data[3]))
        if data[1] == "GET":
            if rospy.has_param("~" + data[2]): 
                response = rospy.get_param("~" + data[2])
            else:
                response = None

        # Send response (either variable or confirmation)
        tcp_connection.send(json.dumps(response))
        tcp_connection.close()


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
        rospy.loginfo("[TCPCommunicationServerNode] Shutdown.")
        self.tcp_socket.close()

if __name__ == '__main__':
    rospy.init_node('tcp_communication_server_node',anonymous=False)
    tcp_communication_server_node = TCPCommunicationServerNode()
    rospy.on_shutdown(tcp_communication_server_node.onShutdown)
    rospy.spin()
