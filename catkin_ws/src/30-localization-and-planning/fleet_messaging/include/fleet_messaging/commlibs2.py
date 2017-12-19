"""Communication module for Duckiebots"""

from __future__ import print_function
import time
import netifaces as ni
import zmq
import libserialize #import the correct seriallibs, see usage below

class DuckieMQ(object):
    """ZMQ implementation for communication between Duckiebots"""
    def __init__(self, interface="wlan0", port="5554", socktype='sub'):
        """Initialzes either a reciever or publisher socket on the specified interface and port.
        Use ifconfig to determin the name of the correct interface.
        On standard Duckiebots the interface is called 'wlan0'.
        Communication works over epgm protocol (broadcasting)"""
        self.context = zmq.Context()
        self.socktype = socktype
        # Uncomment, if needed in future implementations
        # self.port = port
        # self.interface = interface

        self.ownip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
        self.endpoint = "epgm://" + self.ownip + ":" + port
        self.def_filter = 0

        # Rate [kbit] can be setup manually (if not, optimal rate is taken)
        # rate = 40*1000

        if socktype == 'pub':
            self.socket = self.context.socket(zmq.PUB)
            self.socket.connect(self.endpoint)  # should connect
            # self.socket.setsockopt_string(zmq.RATE, rate)
            print('publisher initialized on ' + self.endpoint)

        elif socktype == 'sub':
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, '')
            self.def_filter = 1
            self.socket.connect(self.endpoint)
            #self.socket.setsockopt_string(zmq.RATE, rate)
            print('subscriber initialized on ' + self.endpoint)

        else:
            self.socktype = 'none'
            print('no known type')

        time.sleep(0.2)  # to guarantee initiaization

    def cleanup(self):
        """
        Destroy the sockets.
        """
        self.context.destroy()

    def removefilter(self, filter_string=''):
        """removes filter_string from filter list, default removes default "" string"""
        if self.socktype == 'sub':
            try:
                self.socket.setsockopt(zmq.UNSUBSCRIBE, filter_string)
            except TypeError:
                self.socket.setsockopt_string(zmq.UNSUBSCRIBE, filter_string)
            print('removed filter: \"' + filter_string + '\" on ' + self.endpoint)
        else:
            print("socket not of type subscriber, no filter changed")

    def setfilter(self, filter_string):
        """set filter to only recieve messages starting with filter_string (string)"""
        if self.socktype == 'sub':
            if self.def_filter:
                self.removefilter()
                self.def_filter = 0
            try:
                self.socket.setsockopt(zmq.SUBSCRIBE, filter_string)
            except TypeError:
                self.socket.setsockopt_string(zmq.SUBSCRIBE, filter_string)
            print('set filter: \"' + filter_string + '\" on ' + self.endpoint)
        else:
            print("socket not of type subscriber, no filter changed")

    def send_string(self, msg):
        """send string"""
        if self.socktype == 'pub':
            self.socket.send_string(msg)
        else:
            print("socket not of type publisher, no message sent")

    def send_serialized(self, msg):
        """serialize message with seriallibs.serialize and send"""
        if self.socktype == 'pub':
            self.socket.send_serialized(msg, libserialize.serialize, flags=0, copy=True)
        else:
            print("socket not of type publisher, no message sent")

    def rcv_string(self):
        """receive string"""
        if self.socktype == 'sub':
            return self.socket.recv_string()
        else:
            print("socket not of type subscriber, no message will be recieved")

    def rcv_serialized(self):
        """receive message and deserialize with seriallibs.deserialize"""
        if self.socktype == 'sub':
            return self.socket.recv_serialized(libserialize.parse, flags=0, copy=True)
        else:
            print("socket not of type subscriber, no message will be recieved")
