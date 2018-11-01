#!/usr/bin/env python
## AIDO localization tag_collection
# Author: Chen-Lung (Eric) Lu, ETHZ NCTU, eric565648.eed03@g2.nctu.edu.tw

## This script collect all tf data and transfer them to RemapPose meesage type
# and finally send them to server for system calibration


import rospkg
import rospy
import yaml
import socket
from duckietown_msgs.msg import TagInfo, BoolStamped, RemapPose, RemapPoseArray
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
import tf
# from tf
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class Tag_collection(object):

    def __init__(self):

        self.node_name = "tag_collection"

# -------- Start adding back the tag info stuff
# Code from apriltags_postprocessing_node

        # Save sign tags here
        self.fixed_tags_dict = dict()

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltags_ros')
        tags_filepath = self.setupParam("~tags_file", self.pkg_path+"/../signs_and_tags/apriltagsDB.yaml") # No tags_file input atm., so default value is used
        self.loc = self.setupParam("~loc", -1) # -1 if no location is given
        tags_file = open(tags_filepath, 'r')
        self.tags_dict = yaml.load(tags_file)
        tags_file.close()
        self.info = TagInfo()

        self.sign_types = {"StreetName": self.info.S_NAME,
            "TrafficSign": self.info.SIGN,
            "Light": self.info.LIGHT,
            "Localization": self.info.LOCALIZE,
            "Vehicle": self.info.VEHICLE}

# ---- end tag info stuff

        self.sub_prePros        = rospy.Subscriber("apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)
        self.sub_tf             = tf.TransformListener()
        self.pub_tf             = tf.TransformBroadcaster()
        self.pub_postPros       = rospy.Publisher("~apriltags_out", RemapPoseArray, queue_size=1)

        rospy.loginfo("[%s] has started", self.node_name)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    # Start get tags and publish there relationship through RemapPoseArray message
    def callback(self, msg):
        tag_infos = []

        # Load tag detections message
        for detection in msg.detections:

            # rospy.loginfo("[%s] detection", self.node_name)
            # ------ start tag info processing

            new_info = TagInfo()
            new_info.id = int(detection.id[0])
            id_info = self.tags_dict[new_info.id]



            # Check yaml file to fill in ID-specific information

            new_info.tag_type = self.sign_types[id_info['tag_type']]

            # fixed tags will be added to the database,
            # StreetSigns, TrafficSigns, Localize are considered to be fixed tags
            if (new_info.tag_type == self.info.S_NAME) or (new_info.tag_type == self.info.SIGN) or (new_info.tag_type == self.info.LOCALIZE) or (new_info.tag_type == self.info.LIGHT):
                 # add fixed tag to the database, overwrite old information
                 self.fixed_tags_dict[new_info.id] = [new_info.tag_type, detection.pose]
                 # for fixed_frame in self.fixed_tags_dict:
                 #    rospy.loginfo("FixedFrame: %s",fixed_frame)

#After detecting tags, find transformatinos between them and publish them
        remap_poses_array = RemapPoseArray()
        for parent_frame_tags_id in self.fixed_tags_dict:
            for child_frame_tags_id in self.fixed_tags_dict:
                if parent_frame_tags_id == child_frame_tags_id:
                    # We don't need to know the tf of same tags
                    continue
                else:
                    ## Limit packet size to three poses
                    if len(remap_poses_array.poses) == 3:
                        self.pub_postPros.publish(remap_poses_array)
                        remap_poses_array = RemapPoseArray()

                    child_frame_name = 'Tag'+str(child_frame_tags_id)
                    parent_frame_name = 'Tag'+str(parent_frame_tags_id)
                    if self.sub_tf.frameExists(child_frame_name) and self.sub_tf.frameExists(parent_frame_name):
                        try:
                            t = self.sub_tf.getLatestCommonTime(child_frame_name, parent_frame_name)
                            (trans,rot) = self.sub_tf.lookupTransform(child_frame_name, parent_frame_name, t)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue

                        remap_pose = RemapPose()
                        remap_pose.host = socket.gethostname()
                        remap_pose.frame_id = child_frame_tags_id
                        remap_pose.bot_id = parent_frame_tags_id
                        remap_pose.posestamped.pose.position.x = trans[0]
                        remap_pose.posestamped.pose.position.y = trans[1]
                        remap_pose.posestamped.pose.position.z = trans[2]
                        remap_pose.posestamped.pose.orientation.x = rot[0]
                        remap_pose.posestamped.pose.orientation.y = rot[1]
                        remap_pose.posestamped.pose.orientation.z = rot[2]
                        remap_pose.posestamped.pose.orientation.w = rot[3]
                        #Add this remap pose to the array
                        remap_poses_array.poses.append(remap_pose)

        self.pub_postPros.publish(remap_poses_array)


if __name__ == '__main__':
    rospy.init_node('tag_collection',anonymous=False)
    node = Tag_collection()
    rospy.spin()
