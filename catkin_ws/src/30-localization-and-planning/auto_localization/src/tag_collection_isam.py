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
import global_pose_functions as gposf

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

        self.static_id = [385, 335, 388, 341, 361, 367, 347, 379,317]

# ---- end tag info stuff

        self.sub_prePros        = rospy.Subscriber("apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)
        self.sub_tf             = tf.TransformListener()
        self.pub_tf             = tf.TransformBroadcaster()
        self.pub_postPros       = rospy.Publisher("~apriltags_out", RemapPoseArray, queue_size=1)

        self.pub_switch = rospy.Publisher("apriltag_detector_node/switch", BoolStamped, queue_size=1)
        self.trigger_flag = True

        rospy.loginfo("[%s] has started", self.node_name)
        self.switch_on()

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def switch_on(self):

        rate = rospy.Rate(5)
        while self.trigger_flag:
            trigger = BoolStamped()
            trigger.data = True

            self.pub_switch.publish(trigger)
            rate.sleep()

    # Start get tags and publish there relationship through RemapPoseArray message
    def callback(self, msg):
        tag_infos = []
        remap_poses_array = RemapPoseArray()
        flag_send_msg = False

        # Load tag detections message
        for detection in msg.detections:

            # rospy.loginfo("[%s] detection", self.node_name)
            # ------ start tag info processing

            new_info = TagInfo()
            new_info.id = int(detection.id[0])
            id_info = self.tags_dict[new_info.id]



            # Check yaml file to fill in ID-specific information
            # IF statement checks if Tag is actually in the database
            if not id_info['tag_type']:
                new_info.tag_type = 'Unknown'
                print "Detected unknown Tag with ID: " + str(new_info.id)

            else:
                new_info.tag_type = self.sign_types[id_info['tag_type']]

            # fixed tags will be added to the database,
            # StreetSigns, TrafficSigns are considered to be fixed tags
            # if (new_info.tag_type == self.info.S_NAME) or (new_info.tag_type == self.info.SIGN) or (new_info.tag_type == self.info.SIGN) or (new_info.tag_type == self.info.SIGN):

            # if not new_info.tag_type == self.info.VEHICLE : # We assume any tag doesn't belone to vehicle is reference tags
            if new_info.id in self.static_id:
                 # add fixed tag to the database, overwrite old information

                 self.fixed_tags_dict[new_info.id] = [new_info.tag_type, gposf.get_matrix_from_pose(detection.pose.pose.pose)]
                 # for fixed_frame in self.fixed_tags_dict:
                 #    rospy.loginfo("FixedFrame: %s",fixed_frame)



            # perform coordinate transform for moving tags
            #    in reference to the fixed tags
            elif new_info.id not in self.static_id:

                new_info.vehicle_name = id_info['vehicle_name']
                #rospy.loginfo("Detected %s with Tag ID %s", new_info.vehicle_name, new_info.id)
                # cantrans = tf.canTransform('quacky/color_optical_frame', 'Tag15', now)
                pose = detection.pose.pose.pose
                mat_bot_cam = gposf.get_matrix_from_pose(pose)
                # print "translation bot camera", trans_bot_cam



                # The TCP layer can only send 3 poses at once: send out data every 3 poses:


                # Coordinates transform for each fixed frame
                for fixed_frame in self.fixed_tags_dict:
                    # print "RemapPoseArray size: " + str(len(remap_poses_array.poses))
                    fixed_tag = self.fixed_tags_dict[fixed_frame]
                    if len(remap_poses_array.poses) == 3:
                        self.pub_postPros.publish(remap_poses_array)
                        remap_poses_array = RemapPoseArray()


                    remap_pose = RemapPose()

                    ########## Method 1 Use the pre-defined function gposf ##################

                    # This method is wrong,
                    # mat_bot_tag = gposf.absolute_from_relative_position(mat_bot_cam, fixed_tag[1])
                    # trans, rot = gposf.rot_trans_from_matrix(mat_bot_tag)
                    # rot_euler = tf.transformations.euler_from_quaternion(rot)
                    # rot_euler = [elem * 360 / (2 *3.14159) for elem in rot_euler]
                    # rot_rnd   = ['%.1f' % elem for elem in rot_euler]
                    # trans_rnd = ['%.3f' % elem for elem in trans]
                    #
                    # print "Translation: ", trans_rnd
                    # print "Rotation: ", rot_rnd

                    # get pose of moving tag in reference to the fixed tags
                    ##########################################################################

                    ########## Method 2 Use tf directly ###########################
                    #  in terminal it would be rosrun tf tf_echo fixed_frame bot_frame
                    # how about the time?? rospy.time(0) probably induces an artificial delay
                    try:
                        # Determines that most recent time for which Transformer can compute the transform between
                        # 'Tag'+str(fixed_frame) and 'Tag'+str(new_info.id)
                        t = self.sub_tf.getLatestCommonTime('Tag'+str(fixed_frame), 'Tag'+str(new_info.id))

                        # Switch the position of look up transform
                        # Set parent frame to Duckiebot frame, child frame to fixed frame (tag)
                        # Since we want the transform from Duckiebot to fixed frame (tag)
                        (trans,rot) = self.sub_tf.lookupTransform('Tag'+str(fixed_frame), 'Tag'+str(new_info.id), t)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                    # Publish this as a tf message
                    # self.pub_tf.sendTransform(trans,rot, rospy.Time.now(), new_info.vehicle_name,'Tag'+str(fixed_frame))
                    ###########################################################################

                    #Add data to remap pose message
                    remap_pose.host = socket.gethostname()
                    remap_pose.frame_id = fixed_frame
                    remap_pose.bot_id = new_info.id
                    remap_pose.posestamped.header.stamp = detection.pose.header.stamp
                    remap_pose.posestamped.pose.position.x = trans[0]
                    remap_pose.posestamped.pose.position.y = trans[1]
                    remap_pose.posestamped.pose.position.z = trans[2]
                    remap_pose.posestamped.pose.orientation.x = rot[0]
                    remap_pose.posestamped.pose.orientation.y = rot[1]
                    remap_pose.posestamped.pose.orientation.z = rot[2]
                    remap_pose.posestamped.pose.orientation.w = rot[3]
                    #Add this remap pose to the array
                    remap_poses_array.poses.append(remap_pose)
                    flag_send_msg = True
                    #'''

        if flag_send_msg:
            self.pub_postPros.publish(remap_poses_array) # the array can only contain three poses because packet size is limited to 1024 byte


if __name__ == '__main__':
    rospy.init_node('tag_collection',anonymous=False)
    node = Tag_collection()
    rospy.spin()
