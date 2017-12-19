#!/usr/bin/env python2

import subprocess
import cv2
import zmq
import time
import rospy
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

SERVER_PORT = 7777

# Camera image size
CAMERA_WIDTH = 64
CAMERA_HEIGHT = 64

# Camera image shape
IMG_SHAPE = (CAMERA_WIDTH, CAMERA_HEIGHT, 3)

TIME_STEP_LENGTH = 100


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

bridge = CvBridge()

last_good_img = None


class ImageStuff():
    def __init__(self):
        self.last_good_img = None

    def image_callback(self, msg):
        # print("Received an image!")
        # setattr(msg, 'encoding', '')

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite("/home/lpaull/test.jpg",cv2_img)
        except CvBridgeError, e:
            print(e)
        else:
            #
            # cv2.imwrite('camera_image.jpeg', cv2_img)
            self.last_good_img = cv2_img


imagestuff = ImageStuff()

rospy.init_node('ros_zmq_bridge', anonymous=True)
vel_pub = rospy.Publisher('/shamrock/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=5)
image_topic = "/resize_node/image_resize"
img_sub = rospy.Subscriber(image_topic, Image, imagestuff.image_callback)



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
        print('received motor velocities')
        print(msg['values'])

        vel_cmd = WheelsCmdStamped()
        left, right = tuple(msg['values'])
        vel_cmd.vel_left = left
        vel_cmd.vel_right = right
        vel_pub.publish(vel_cmd)
    elif msg['command'] == 'reset':
        pass
    else:
        assert False, "unknown command"

    # only resize when we need
    img = cv2.resize(imagestuff.last_good_img, (CAMERA_WIDTH, CAMERA_HEIGHT))

    # BGR to RGB
    img = img[:, :, ::-1]

    # to contiguous, otherwise ZMQ will complain
    img = np.ascontiguousarray(img, dtype=np.uint8)

    sendArray(socket, img)


for message in poll_socket(socket):
    handle_message(message)
