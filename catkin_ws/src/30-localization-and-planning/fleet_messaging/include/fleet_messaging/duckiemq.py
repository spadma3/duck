"""
Communication module for Duckiebots
"""

# Import modules
from __future__ import print_function
import time
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
        self.__socket.send_string(msg)

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
        self.__socket.send_serialized(msg, libserialize.serialize, flags=0,
                                      copy=True)


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
        self.__filters = []
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

    def addfilter(self, filter_string):
        """
        Add filter to only receive messages starting with filter_string
        (string).
        Inputs:
        - filter_string: string to be filtered
        Outputs:
        None
        """
        try:
            self.__socket.setsockopt(zmq.SUBSCRIBE, filter_string)
        except TypeError:
            self.__socket.setsockopt_string(zmq.SUBSCRIBE, filter_string)
            print("Set filter: \"" + filter_string + "\" on " + self.__endpoint)
        finally:
            self.__filters.append(filter_string)

    def removefilter(self, filter_string=""):
        """
        Removes filter_string from filter list, default removes default ""
        string.
        Inputs:
        - filter_string: string filter to be removed
        Outputs:
        None
        Exceptions:
        - ValueError: if filter_string is not in self.__filters
        """
        try:
            self.__filters.remove(filter_string)
            self.__socket.setsockopt(zmq.UNSUBSCRIBE, filter_string)
        except ValueError:
            raise ValueError("filter_string not set in object.")
        except TypeError:
            self.__socket.setsockopt_string(zmq.UNSUBSCRIBE, filter_string)
            print("Removed filter: \"" + filter_string + "\" on " +
                  self.__endpoint)

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
        if self.__timeout > 0:
            msg = self.rcv_string_timeout()
        else:
            msg = self.__socket.recv_string()

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
        events = self.__poller.poll(self.__timeout)
        if not events:
            msg = None
        else:
            msg = self.__socket.recv_string()

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
        if self.__timeout > 0:
            msg = self.rcv_serialized_timeout()
        else:
            msg = self.__socket.recv_serialized(libserialize.parse, flags=0,
                                                copy=True)

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
        events = self.__poller.poll(self.__timeout)
        if not events:
            msg = None
        else:
            msg = self.__socket.recv_serialized(libserialize.parse, flags=0,
                                                copy=True)

        return msg
