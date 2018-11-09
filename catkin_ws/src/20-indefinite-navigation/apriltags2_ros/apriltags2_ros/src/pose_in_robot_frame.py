#!/usr/bin/env python
import rospkg
import rospy
import yaml
import numpy as np

from duckietown_msgs.msg import RemapPose, RemapPoseArray
from duckietown_utils import tcp_communication
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

# Read clients (watchtower), read from yaml that describe the map in the future.

class AprilTagPostProcesser(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        # Subscribers
        self.sub_tag_detection = rospy.Subscriber("/ata/tag_detections", AprilTagDetectionArray, self.cbTagDetection, queue_size=1)

    def cbTagDetection(self,tag_detection_msg):

        pose = RemapPose()

        pose.bot_id = tag_detection_msg.detections[0].id
        pose.posestamped.header.stamp.secs = tag_detection_msg.detections[0].pose.header.stamp.secs
        pose.posestamped.header.stamp.nsecs = tag_detection_msg.detections[0].pose.header.stamp.nsecs
        pose.posestamped.pose.position.x = tag_detection_msg.detections[0].pose.pose.pose.position.x
        pose.posestamped.pose.position.y = tag_detection_msg.detections[0].pose.pose.pose.position.y
        pose.posestamped.pose.position.z = tag_detection_msg.detections[0].pose.pose.pose.position.z
        pose.posestamped.pose.orientation.x = tag_detection_msg.detections[0].pose.pose.pose.orientation.x
        pose.posestamped.pose.orientation.y = tag_detection_msg.detections[0].pose.pose.pose.orientation.y
        pose.posestamped.pose.orientation.z = tag_detection_msg.detections[0].pose.pose.pose.orientation.z
        pose.posestamped.pose.orientation.w = tag_detection_msg.detections[0].pose.pose.pose.orientation.w

        return pose

if __name__ == '__main__':

    rospy.init_node('april_tag_post_processor',anonymous=False)

    node = AprilTagPostProcesser()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
