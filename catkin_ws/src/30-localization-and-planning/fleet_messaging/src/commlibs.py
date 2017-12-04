# Import modules
import zmq
#import seriallibs
import time
import netifaces as ni

# zeroMQ
class duckie0mq(object):
    # Constructor
    def __init__(self, subip = "192.168.40.10", port = "5555", type = 'sub'):
        self.context = zmq.Context()
        self.type = type
        self.subip = subip
        self.port = port
        if type=='pub':
            self.endpoint = "tcp://" + self.subip + ":" + self.port
            self.socket = self.context.socket(zmq.PUB)
            self.socket.connect(self.endpoint)
            print('publisher initialized on port ' + self.endpoint)
        elif type=='sub':
            self.endpoint = "tcp://*" + ":" + str(self.port)
            self.socket = self.context.socket (zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, '')
            self.socket.bind(self.endpoint)
            print('subscriber initialized on port ' + self.endpoint)
        else:
            self.type = 'none'
            print('no known type')
        time.sleep(0.2)

    # set filter to only recieve messages starting with fltr (string)
    def setfilter(self, filter):
        if self.type == 'sub':
            self.socket.setsockopt_string(zmq.SUBSCRIBE, filter)
            print('set filter: \"'+filter+'\" on '+self.endpoint)
        else:
            print("socket not of type subscriber, no filter changed")

    # connect to additional ports
    def connect(self, port):
        if self.type == 'sub':
            self.endpoint = self.endpoint +', '+ "tcp://*" + ":" + port
            self.socket.bind("tcp://*" + ":" + port)
            print('connected to port: \"' + port)
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
