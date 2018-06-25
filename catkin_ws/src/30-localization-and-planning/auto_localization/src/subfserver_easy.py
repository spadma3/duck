#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import RemapPose, RemapPoseArray
from duckietown_utils import tcp_communication
import numpy as np
from geometry_msgs.msg import PoseStamped

# Read clients (watchtower), read from yaml that describe the map in the future.
watchtowers = ['watchtower01', 'watchtower02', 'watchtower03', 'watchtower04', 'watchtower05', 'watchtower06', 'watchtower07', 'watchtower08']

def poselist2pose(poselist):
    pose = RemapPose()
    pose.host = poselist[0]
    pose.frame_id = poselist[1]
    pose.bot_id = poselist[2]
    pose.posestamped.pose.position.x = poselist[3][0]
    pose.posestamped.pose.position.y = poselist[3][1]
    pose.posestamped.pose.position.z = poselist[3][2]
    pose.posestamped.pose.orientation.x = poselist[4][0]
    pose.posestamped.pose.orientation.y = poselist[4][1]
    pose.posestamped.pose.orientation.z = poselist[4][2]
    pose.posestamped.pose.orientation.w = poselist[4][3]

    return pose

def pubPoses():

    pub_poses = rospy.Publisher('local_poses', RemapPoseArray, queue_size=1)
    rate = rospy.Rate(5) # 5hz

    while not rospy.is_shutdown():
        local_poses_pub = RemapPoseArray()

        for hosts in watchtowers:

            local_poses_list = tcp_communication.getVariable(hosts)
            if local_poses_list == None:
                continue
            else:
                for poses in local_poses_list:
                    local_pose = poselist2pose(poses)
                    local_poses_pub.poses.append(local_pose)

        pub_poses.publish(local_poses_pub)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('subfserver_easy',anonymous=False)
    try:
        pubPoses()
    except rospy.ROSInterruptException:
        pass
