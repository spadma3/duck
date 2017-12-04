#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from duckietown_utils import rgb_from_ros
from PIL import Image
import numpy as np


class LaneFollowerNode(object):
    def __init__(self):
        self.node_name = "Lane Follower"        
        robot_name = rospy.get_param("~robot_name", "")

        # Subscribers
        self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage,
                                           self.callback)

        # Publisher
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped,
                                           queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.timer = rospy.Timer(
            rospy.Duration.from_sec(1.0), self.getGains_event)
        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.subscriber.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd(car_control_msg)
        rospy.sleep(0.5)  # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

    def callback(self, compressed_image):
        img = rgb_from_ros(compressed_image)
        img = Image.fromarray(img)
        img.thumbnail((80, 60))
        img = np.asarray(img)

        '''
        Call tensorflow code and get outputs v, omega
        '''

        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - compressed_image.header.stamp

        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = image_delay_stamp
        car_control_msg.v = 1.
        car_control_msg.omega = 0.5

        self.pub_car_cmd(car_control_msg)


if __name__ == '__main__':
    rospy.init_node('lane_follower',anonymous=False)
    lane_follower_node = LaneFollowerNode()
    rospy.on_shutdown(lane_follower_node.onShutdown)
    rospy.spin()
