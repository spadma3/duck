# Import modules
import zmq
import seriallibs

# zeroMQ
class duckie0mq(object):
    # Constructor
    def __init__(self, port = "tcp://127.0.0.1:5555", type = 'sub', broadcast = 0):
        self.context = zmq.Context()
        self.type = type
        self.broadcast = broadcast
        self.port = port
        if type=='pub':
            self.socket = self.context.socket(zmq.PUB)
            self.socket.connect(port)
            print('publisher initialized on port '+self.port)
        elif type=='sub':
            self.socket = self.context.socket (zmq.SUB)
            self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
            if self.broadcast:
                self.socket.connect(port)
            else:
                self.socket.bind(port)
            print('subscriber initialized on port '+self.port)
        else:
            self.type = 'none'
            print('no known type')
        time.sleep(0.2)

    # set filter to only recieve messages starting with fltr (string)
    def setfilter(self, filter):
        if self.type == 'sub':
            self.socket.setsockopt_string(zmq.SUBSCRIBE, filter)
            print('set filter: \"'+filter+'\" on port '+self.port)
        else:
            print("socket not of type subscriber, no filter changed")

    # connect to additional ports
    def connect(self, port):
        if self.type == 'sub':
            self.port = self.port +', '+ port
            self.socket.bind(port)
            print('connected to port: \"'+port)
        else:
            print("socket not of type subscriber, no port opened")

    # send
    def send_string(self, msg):
        if self.type == 'pub':
            self.socket.send_string(msg)
        else:
            print("socket not of type publisher, no message sent")

    def send_serialized(self, msg):
        if self.type == 'pub':
            self.socket.send_serialized(msg, seriallibs.serialize, flags=0, copy=True)
        else:
            print("socket not of type publisher, no message sent")

    # recieve
    def rcv_string(self):
        if self.type == 'sub':
            return self.socket.recv_string()
        else:
            print("socket not of type subscriber, no message will be recieved")

    def rcv_serialized(self):
        if self.type == 'sub':
            return self.socket.recv_serialized(seriallibs.deserialize, flags=0, copy=True)
        else:
            print("socket not of type subscriber, no message will be recieved")
