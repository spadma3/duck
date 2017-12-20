#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from duckietown_utils import rgb_from_ros
from PIL import Image
import numpy as np
from mvnc import mvncapi as mvnc
from collections import deque
import os

class LaneFollowerNode(object):
    def __init__(self):
        self.node_name = "Imitation Learner"        

        # Subscribers
        self.sub_topic = '/duckduckgo/camera_node/image/compressed'
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage,
                                           self.callback)

        # Publisher
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped,
                                           queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        mvnc.SetGlobalOption(mvnc.GlobalOption.LOG_LEVEL, 2)
        devices = mvnc.EnumerateDevices()
        if len(devices) == 0:
            rospy.loginfo("No devices found!")
	self.image_deque = deque(maxlen=4)
        self.device = mvnc.Device(devices[0])
        self.device.OpenDevice()
        with open('/home/rithesh/duckietown/catkin_ws/src/10-lane-control/imitation_learning/graph', mode='rb') as f:
            blob = f.read()
        self.graph = self.device.AllocateGraph(blob)

        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.subscriber.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(car_control_msg)
        rospy.sleep(0.5)  # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

    def callback(self, compressed_image):
        img = rgb_from_ros(compressed_image)
        img = Image.fromarray(img)
        img.thumbnail((80, 60))
        img = np.asarray(img.convert('L'))

        output = [0.] 
        if hasattr(self, 'image_deque'):
            self.image_deque.append(img)

            if len(self.image_deque) == 4 and hasattr(self, 'graph'):
                batch_img = np.asarray(self.image_deque)
                batch_img = batch_img.reshape(4, 60, 80)
                batch_img = (2*(batch_img/255.) - 1.).astype('float32')
                batch_img = batch_img.transpose(1, 2, 0)

                out = self.graph.LoadTensor(batch_img.astype(np.float16), 'user object')
                assert out is True
                output, userobj = self.graph.GetResult()
                # print "Result: ", output[0]

        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - compressed_image.header.stamp

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = image_delay_stamp
        car_control_msg.v = 0.386
        car_control_msg.omega = output[0]
        self.pub_car_cmd.publish(car_control_msg)

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('imitation_learner', anonymous=False)
    lane_follower_node = LaneFollowerNode()
    rospy.on_shutdown(lane_follower_node.onShutdown)
    rospy.spin()
