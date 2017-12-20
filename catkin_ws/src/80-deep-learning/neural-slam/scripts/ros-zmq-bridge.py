#!/usr/bin/env python2

import subprocess
import zmq
import time
import rospy

### TODO: What are the messages to import?

import numpy as np

SERVER_PORT = 7777

import signal
import sys
def signal_handler(signal, frame):
    print ("exiting")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


def sendArray(socket, array):
    """Send a numpy array with metadata over zmq"""
    md = dict(
        dtype=str(array.dtype),
        shape=array.shape,
    )
    # SNDMORE flag specifies this is a multi-part message
    socket.send_json(md, flags=zmq.SNDMORE)
    return socket.send(array, flags=0, copy=True, track=False)


print('Starting up')
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://*:%s" % SERVER_PORT)


class GetObservation():
    def __init__(self):
        self.got_new_observation = False
        self.observation = None
        
    def image_callback(self, msg):
        print("Received an observation!")

        
        # TODO -> need to parse the message into a numpy arraw or something to be transmitted
        self.observation = msg.data


get_observation_obj = GetObservation()

rospy.init_node('ros_zmq_bridge', anonymous=True)

### TODO: Exactly what are we sending to ROS
pub = rospy.Publisher('/shamrock/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=5)
### TODO: Exactly what are we getting from ROS
isub = rospy.Subscriber(image_topic, Image, imagestuff.image_callback)



# waiting for ROS to connect... TODO solve this with ROS callback
time.sleep(2)

def poll_socket(socket, timetick = 10):
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    # wait up to 10msec
    try:
        print("poller ready")
        while True:
            obj = dict(poller.poll(timetick))
            if socket in obj and obj[socket] == zmq.POLLIN:
                yield socket.recv_json()
    except KeyboardInterrupt:
        print ("stopping server")
        quit()

def handle_message(msg):
    if msg['command'] == 'action':
        print('received action')
        print(msg['values'])

        ### TODO Build a message and send it
        pub.publish(vel_cmd)
    elif msg['command'] == 'reset':
        ## TODO: Do we need to do anything on a reset?
        pass
    else:
        assert False, "unknown command"

    while not get_observation_obj.got_new_observation:
        print "waiting for observation"
        time.sleep(0.5)

    print "got observation"
    sendArray(socket, get_observation_obj.observation)
    get_observation_obj.got_new_observation = False
    

for message in poll_socket(socket):
    handle_message(message)
