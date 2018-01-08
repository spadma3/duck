#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm
import tf_func as tff

from duckietown_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point, Pose, PoseStamped

rospy.init_node('target2bot_apriltags', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()

robot_tags = 11
obj_tags = 12
obj_bias = 0.034

def apriltag_callback(data):
    #print "Get data"

    obj_point = rospy.wait_for_message('/homohrt/depth_cir/obj_point', Point, timeout=None)

    tp_pub = rospy.Publisher("~tar_pose", Pose, queue_size=1)

    # use apriltag pose detection to find where is the robot
    if len(data.detections)!=0:  # check if apriltag is detected

        robot_get = False
        obj_get = False
    
        for i in range(len(data.detections)):
            detection = data.detections[i]
            #print "ID: ", detection.id, "; Pose: ", detection.pose

            if detection.id == robot_tags:
                robot_get = True
                robot_pose = tff.pose2poselist(detection.pose.pose)
                tff.pubFrame(br, robot_pose, 'robot_arm', 'camera_link')
            '''
            elif detection.id == obj_tags:
                obj_get = True
                #make tar_pose from apriltags to duckie's "head"
                obj_pose = detection.pose.pose
                obj_pose.position.x = obj_pose.position.x - obj_bias * np.sin(obj_pose.orientation.z)
                obj_pose.position.z = obj_pose.position.z + obj_bias * np.cos(obj_pose.orientation.z)
                #tar_pose = tff.pose2poselist(detection.pose.pose)
                tar_pose = tff.pose2poselist(obj_pose)
                tar_pose = tff.transformPose(lr, tar_pose, 'camera_link', 'robot_arm')
                tar_pose_pub = tff.poselist2pose(tar_pose)
                if robot_get == True:
                    tp_pub.publish(tar_pose_pub)
            '''
        #print "done apriltag detection"
        if robot_get == True:
            #print "start target pose get"
            obj_pose = Pose()
            obj_pose.position.x = obj_point.x
            obj_pose.position.y = obj_point.y
            obj_pose.position.z = obj_point.z
            obj_pose.orientation.x = 0
            obj_pose.orientation.y = 0
            obj_pose.orientation.z = 0
            obj_pose.orientation.w = 1

            tar_pose = tff.pose2poselist(obj_pose)
            tar_pose = tff.transformPose(lr, tar_pose, 'camera_link', 'robot_arm')
            tar_pose_pub = tff.poselist2pose(tar_pose)
            tp_pub.publish(tar_pose_pub)


if __name__ == '__main__':
    print "Get Start"

    apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    #obj_detect_sub = rospy.Subscriber("")

    rospy.sleep(1)
    
    rospy.spin()

