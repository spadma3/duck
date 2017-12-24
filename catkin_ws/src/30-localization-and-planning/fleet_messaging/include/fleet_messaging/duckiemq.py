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


class DuckieMQBase(object):
    """
    Base class for ZeroMQ communication between Duckiebots.
    """
    def __init__(self, interface="wlan0", port="5554"):
        """
        Initializes the ZeroMQ contect.
        Use ifconfig to determine the name of the correct interface.
        On standard Duckiebots the interface is called "wlan0".
        Communication works over epgm protocol (multicasting)
        Inputs:
        - interface: Interface used by the socket
        - port:      Port used by the socket
        Ouptuts:
        - object: Duckietown ZeroMQ communication object
        """
        self.__context = zmq.Context()
        ownip = ni.ifaddresses(interface)[ni.AF_INET][0]["addr"]
        self.__endpoint = "epgm://" + ownip + ":" + port
        self.__def_filter = False
        # Uncomment, if needed in future implementations
        # self.__interface = interface
        # self.__ownip = ownip
        # self.__port = port

        # Rate [kbit] can be setup manually (if not, optimal rate is taken)
        # self.__rate = 40*1000

        # Sleep, to guarantee initialization
        time.sleep(0.2)

    def __del__(self):
        """
        Destructor. Destroy the ZeroMQ context.
        """
        self.__context.destroy()


class DuckieMQSender(DuckieMQBase):
    """
    DuckieMQ sender class.
    """
    def __init__(self, interface="wlan0", port="5554"):
        """
        Initializes a sender socket on the specified interface and port.
        Use ifconfig to determine the name of the correct interface.
        On standard Duckiebots the interface is called "wlan0".
        Communication works over epgm protocol (multicasting)
        Inputs:
        - interface: Interface used by the socket
        - port:      Port used by the socket
        Ouptuts:
        - object: ZeroMQ socket
        """
        # Call the base constructor
        super(DuckieMQSender, self).__init__(interface, port)

        # Configure sender socket
        self.__socket = self.__context.socket(zmq.PUB)
        self.__socket.connect(self.__endpoint)  # should connect
        # self.socket.setsockopt_string(zmq.RATE, self.__rate)
        print("Publisher initialized on " + self.__endpoint)

        # Sleep, to guarantee initialization
        time.sleep(0.2)


class DuckieMQReceiver(DuckieMQBase):
    """
    DuckieMQ receiver class.
    """
    def __init__(self, interface="wlan0", port="5554", timeout=0):
        """
        Initializes a receiver socket on the specified interface and port.
        Use ifconfig to determine the name of the correct interface.
        On standard Duckiebots the interface is called "wlan0".
        Communication works over epgm protocol (multicasting)
        Inputs:
        - interface: Interface used by the socket
        - port:      Port used by the socket
        - timeout:   timeout on receiving messages
        Ouptuts:
        - object: ZeroMQ socket
        Exceptions:
        - ValueError:   if timeout is smaller than 0
        """
        # Call the base constructor
        super(DuckieMQReceiver, self).__init__(interface, port)

        # Configure receiver socket
        self.__socket = self.__context.socket(zmq.SUB)
        self.__socket.setsockopt(zmq.SUBSCRIBE, "")
        self.__def_filter = False
        self.__socket.connect(self.__endpoint)
        #self.socket.setsockopt_string(zmq.RATE, self.rate)
        if timeout < 0:
            raise ValueError("timeout must be greater or equal 0!")
        if timeout > 0:
            self.__poller = zmq.Poller()
            self.__poller.register(self.__socket, zmq.POLLIN)
        self.__timeout = timeout
        print("Subscriber initialized on " + self.__endpoint)

        # Sleep, to guarantee initialization
        time.sleep(0.2)

    def __del__(self):
        """
        Destructor. Unregister the poller.
        """
        # Unregister the poller
        if self.__timeout > 0:
            self.__poller.unregister(self.__socket)

        # Call the parent destructor
        super(DuckieMQReceiver, self).__del__()

    def setfilter(self, filter_string):
        """
        Set filter to only recieve messages starting with filter_string
        (string).
        Inputs:
        - filter_string: string to be filtered
        Outputs:
        None
        """
        if self.__def_filter:
            self.removefilter()
            self.__def_filter = True
        try:
            self.__socket.setsockopt(zmq.SUBSCRIBE, filter_string)
        except TypeError:
            self.__socket.setsockopt_string(zmq.SUBSCRIBE, filter_string)
            print("Set filter: \"" + filter_string + "\" on " + self.__endpoint)

    def removefilter(self, filter_string=""):
        """
        Removes filter_string from filter list, default removes default ""
        string.
        Inputs:
        - filter_string: string filter to be removed
        Outputs:
        None
        """
        try:
            if self.__def_filter:
                self.__socket.setsockopt(zmq.UNSUBSCRIBE, filter_string)
                self.__def_filter = False
        except TypeError:
            self.__socket.setsockopt_string(zmq.UNSUBSCRIBE, filter_string)
            print("Removed filter: \"" + filter_string + "\" on " +
                  self.__endpoint)


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
        if self.socktype == "sub" and self.timeout > 0:
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
