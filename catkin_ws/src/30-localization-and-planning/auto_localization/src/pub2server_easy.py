#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import RemapPose, RemapPoseArray
from duckietown_utils import tcp_communication
import numpy as np
from geometry_msgs.msg import PoseStamped

def pose2poselist(pose):
    poselist = []
    poselist.append(pose.position.x)
    poselist.append(pose.position.y)
    poselist.append(pose.position.z)
    poselist.append(pose.orientation.x)
    poselist.append(pose.orientation.y)
    poselist.append(pose.orientation.z)
    poselist.append(pose.orientation.w)

    return poselist

def cbPose(msg):

    poses2server = []
    for pose in msg.poses:
        pub_pose = []
        pub_pose.append(pose.host)
        pub_pose.append(pose.frame_id)
        pub_pose.append(pose.bot_id)
        poselist = pose2poselist(pose.posestamped.pose)
        pub_pose.append(poselist[:3])
        pub_pose.append(poselist[3:])
        poses2server.append(pub_pose)
    print "pub2server: ", poses2server
    tcp_communication.setVariable("watchtower01", poses2server)


if __name__ == '__main__':
    rospy.init_node('pub2server_easy',anonymous=False)
    sub_msg = rospy.Subscriber("apriltags_out", RemapPoseArray, cbPose)
    rospy.spin()
