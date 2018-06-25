#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import RemapPose, RemapPoseArray
from duckietown_utils import tcp_communication
import numpy as np
from geometry_msgs.msg import PoseStamped

# Read clients (watchtower), read from yaml that describe the map in the future.
machines = ['watchtower01', 'watchtower02', 'watchtower03', 'watchtower04', 'watchtower05', 'watchtower06', 'watchtower07', 'watchtower08']

def poselist2pose(poselist):
    pass

def pubPoses():

    poses = RemapPoseArray()
    poses_from_server =

    for hosts in machines:

        local_poses = tcp_communication.getVariable(hosts)
        if local_poses == None:
            continue
        else:
            print "Local:", local_poses


if __name__ == '__main__':
    rospy.init_node('subfserver_easy',anonymous=False)
    try:
        pubPoses()
    except rospy.ROSInterruptException:
        pass
