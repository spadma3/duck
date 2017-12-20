"""
Communication module for Duckiebots
"""

# Import modules
from __future__ import print_function
import time
import socket
import netifaces as ni
import libserialize
import zmq

class DuckieMQ(object):
    """
    ZeroMQ class implementation for communication between Duckiebots.
    """
    def __init__(self, interface="wlan0", port="5554", socktype="sub",
                 timeout=0):
        """
        Initialzes either a reciever or publisher socket on the specified
        interface and port.
        Use ifconfig to determine the name of the correct interface.
        On standard Duckiebots the interface is called "wlan0".
        Communication works over epgm protocol (broadcasting)
        Inputs:
        - interface: Interface used by the socket
        - port:      Port used by the socket
        - socktype:  Socket type ("sub"/"pub")
        - timeout:   timeout on receiving messages
        Ouptuts:
        - object: ZeroMQ socket
        Exceptions:
        - ValueError:   if timeout smaller than 0
        - socket.error: if socket type neither "pub" nor "sub"
        """
        self.context = zmq.Context()
        self.ownip = ni.ifaddresses(interface)[ni.AF_INET][0]["addr"]
        self.port = port
        self.def_filter = False
        endpoint = self.build_endpoint()
        # Uncomment, if needed in future implementations
        # self.interface = interface

        # Rate [kbit] can be setup manually (if not, optimal rate is taken)
        # rate = 40*1000

        # Configure publisher socket
        if socktype == "pub":
            self.socket = self.context.socket(zmq.PUB)
            self.socket.connect(endpoint)  # should connect
            # self.socket.setsockopt_string(zmq.RATE, rate)
            print("Publisher initialized on " + endpoint)

        # Configure subscriber socket
        elif socktype == "sub":
            self.socket = self.context.socket(zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, "")
            self.def_filter = True
            self.socket.connect(endpoint)
            #self.socket.setsockopt_string(zmq.RATE, rate)
            if timeout < 0:
                raise ValueError("timeout must be greater or equal 0!")
            if timeout > 0:
                self.poller = zmq.Poller()
                self.poller.register(self.socket, zmq.POLLIN)
            print("Subscriber initialized on " + endpoint)

        else:
            raise socket.error("No known socket type!")

        # Set the socket type and timeout
        self.socktype = socktype
        self.timeout = timeout

        # Sleep, to guarantee initialization
        time.sleep(0.2)

    def cleanup(self):
        """
        Destroy the socket.
        """
        # Unregister the poller
        if self.socktype == "sub":
            self.poller.unregister(self.socket)

        self.context.destroy()

    def removefilter(self, filter_string=""):
        """
        Removes filter_string from filter list, default removes default ""
        string.
        Inputs:
        - filter_string: string filter to be removed
        Outputs:
        None
        Exceptions:
        - socket.error: if socket type is not "sub"
        """
        if self.socktype == "sub":
            try:
                self.socket.setsockopt(zmq.UNSUBSCRIBE, filter_string)
            except TypeError:
                self.socket.setsockopt_string(zmq.UNSUBSCRIBE, filter_string)
                print("Removed filter: \"" + filter_string + "\" on " +
                      self.build_endpoint())
        else:
            raise socket.error("Socket not of type subscriber, no filter " +
                               "changed.")

    def setfilter(self, filter_string):
        """
        Set filter to only recieve messages starting with filter_string
        (string).
        Inputs:
        - filter_string: string to be filtered
        Outputs:
        None
        Exceptions:
        - socket.error: if socket type is not "sub"
        """
        if self.socktype == "sub":
            if self.def_filter:
                self.removefilter()
                self.def_filter = False
            try:
                self.socket.setsockopt(zmq.SUBSCRIBE, filter_string)
            except TypeError:
                self.socket.setsockopt_string(zmq.SUBSCRIBE, filter_string)
                print("Set filter: \"" + filter_string + "\" on " +
                      self.build_endpoint())
        else:
            raise socket.error("Socket not of type subscriber, no filter " +
                               "changed.")

    def send_string(self, msg):
        """
        Send a string through the socket.
        Inputs:
        - msg: string to be sent through the socket
        Outputs:
        None
        Exceptions:
        - socket.error: if socket type is not "pub"
        """
        if self.socktype == "pub":
            self.socket.send_string(msg)
        else:
            raise socket.error("Socket not of type publisher, no message sent.")

    def send_serialized(self, msg):
        """
        Serialize message with libserialize.serialize and send it.
        Inputs:
        - msg: Message to be serialized and sent
        Outputs:
        None
        Exceptions:
        - socket.error: if socket type is not "pub"
        """
        if self.socktype == "pub":
            self.socket.send_serialized(msg, libserialize.serialize, flags=0,
                                        copy=True)
        else:
            raise socket.error("Socket not of type publisher, no message sent.")

    def rcv_string(self):
        """
        Receive string through the socket.
        Inputs:
        None
        Outputs:
        msg: Received string
        Exceptions:
        - socket.error: if socket type is not "sub"
        """
        if self.socktype == "sub":
            if self.timeout > 0:
                msg = self.rcv_string_timeout()
            else:
                msg = self.socket.recv_string()
        else:
            raise socket.error("Socket not of type subscriber , no message " +
                               "will be received.")

        return msg

    def rcv_string_timeout(self):
        """
        Receive string through the socket using a timeout.
        Inputs:
        - timeout: timeout in seconds
        Outputs:
        msg: Received string (None if timeout occurs, before any msg arrives)
        """
        # Poll for a message
        events = self.poller.poll(self.timeout)
        if not events:
            msg = None
        else:
            msg = self.socket.recv_string()

        return msg

    def rcv_serialized(self):
        """
        Receive message and deserialize with libserialize.deserialize.
        Inputs:
        None
        Outputs:
        msg: Received parsed string
        Exceptions:
        - socket.error: if socket type is not "sub"
        """
        if self.socktype == "sub":
            if self.timeout > 0:
                msg = self.rcv_serialized_timeout()
            else:
                msg = self.socket.recv_serialized(libserialize.parse, flags=0,
                                                  copy=True)
        else:
            raise socket.error("Socket not of type subscriber or using " +
                               "timeout, no message will be received.")

        return msg

    def rcv_serialized_timeout(self):
        """
        Receive message and deserialize with libserialize.deserialize.
        Inputs:
        - timeout: timeout in seconds
        Outputs:
        msg: Received parsed message (None if timeout occurs, before any msg
             arrives)
        """
        # Poll for a message
        events = self.poller.poll(self.timeout)
        if not events:
            msg = None
        else:
            msg = self.socket.recv_serialized(libserialize.parse, flags=0,
                                              copy=True)

        return msg

    def build_endpoint(self):
        """
        Build an endpoint string.
        Inputs:
        None
        Outputs:
        endpoint string
        """
        return "epgm://" + self.ownip + ":" + self.port
