# Import modules
import zmq
#import seriallibs
import time
import netifaces as ni

# zeroMQ
class duckie0mq(object):
    # Constructor
    def __init__(self, interface = "wlan0", port = "5554", type = 'sub'):

        self.context = zmq.Context()
        self.type = type
        self.port = port

        self.ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
        self.endpoint = "epgm://" + self.ip + ":" + self.port

        if type=='pub':
            self.socket = self.context.socket(zmq.PUB)
            self.socket.connect(self.endpoint) #should connect
            print('publisher initialized on '+self.endpoint)

        elif type=='sub':
            self.socket = self.context.socket (zmq.SUB)
            self.socket.setsockopt(zmq.SUBSCRIBE, '')
            self.socket.connect(self.endpoint)
            print('subscriber initialized on '+self.endpoint)

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
