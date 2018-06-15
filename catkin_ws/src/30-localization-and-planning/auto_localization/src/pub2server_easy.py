#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import RemapPose, RemapPoseArray
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from duckietown_utils import tcp_communication
import numpy as np
import tf
# from tf
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

def cbPose(msg):

    poses2server = []
    for pose in msg.poses:
        pub_pose = []
        pub_pose.append(pose.frame_id)
        pub_pose.append(pose.bot_id)
        pub_pose.append(pose.posestamped.pose)
        poses2server.append(pub_pose)
    tcp_communication.setVariable("watchtower01", poses2server)


if __name__ == '__main__':
    rospy.init_node('pub2server_easy',anonymous=False)
    sub_msg = rospy.Subscriber("apriltags_out", RemapPoseArray, cbPose)
    rospy.spin()
