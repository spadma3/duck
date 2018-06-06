#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, BoolStamped
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
import tf
# from tf
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped


# This node localizes Apriltags prior defined in
# Coordinates will be given in
# Subscribes:   Apriltag detections (tf)
# Publishes:    Coordinates of BotAprilTag in Reference to FixedTags as tf_message

class AprilLocalLocalization(object):
    """ """
    def __init__(self):
        """ """
        self.node_name = "apriltags2_local_localization_node"

        # Constants
        self.world_frame = "world"
        self.camera_frame = "camera"
        self.duckiebot_frame = "duckiebot"

        # Number of fixed AprilTags detected in the field of view
        self.fixed_tags_number = 0

        # This dictionary contains information of the fixed tags
        # {TagID : [Type | detection.pose]}
        self.fixed_tags_dict = dict()
        # Setup the publishers and subscribers from localization_node
        # self.sub_april = rospy.Subscriber("~apriltags", AprilTagsWithInfos, self.tag_callback)
        # self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1, latch=True)


# -------- Start adding back the tag info stuff
# Code from apriltags_postprocessing_node

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
        self.pub_tf = tf.TransformBroadcaster()
        self.pub_postPros       = rospy.Publisher("~apriltags_out", AprilTagsWithInfos, queue_size=1)
        # self.pub_visualize      = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)

        rospy.loginfo("[%s] has started", self.node_name)



    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, msg):
        tag_infos = []

        # Load tag detections message
        for detection in msg.detections:

            rospy.loginfo("[%s] detection", self.node_name)
            # ------ start tag info processing
            # rospy.loginfo("detection.id [%s]", detection.id[0])
            new_info = TagInfo()
            new_info.id = int(detection.id[0])
            id_info = self.tags_dict[new_info.id]
            #rospy.loginfo(id_info)


            # Check yaml file to fill in ID-specific information

            new_info.tag_type = self.sign_types[id_info['tag_type']]

            # fixed tags will be added to the database,
            # StreetSigns, TrafficSigns are considered to be fixed tags
            if (new_info.tag_type == self.info.S_NAME) or (new_info.tag_type == self.info.SIGN):
                 # add fixed tag to the database, overwrite old information
                 self.fixed_tags_dict[new_info.id] = [new_info.tag_type, detection.pose]
                 # for fixed_frame in self.fixed_tags_dict:
                 #    rospy.loginfo("FixedFrame: %s",fixed_frame)



            # perform coordinate transform for moving tags
            #    in reference to the fixed tags
            elif new_info.tag_type == self.info.VEHICLE:
                new_info.vehicle_name = id_info['vehicle_name']
                rospy.loginfo("Detected %s with Tag ID %s", new_info.vehicle_name, new_info.id)
                # cantrans = tf.canTransform('quacky/color_optical_frame', 'Tag15', now)


                # Coordinates transform for each fixed frame
                for fixed_frame in self.fixed_tags_dict:


                    #  in terminal it would be rosrun tf tf_echo fixed_frame bot_frame
                    # how about the time, does it work this way?
                    try:
                        (trans,rot) = self.sub_tf.lookupTransform('Tag'+str(new_info.id), 'Tag'+str(fixed_frame), rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                    # Publish this as a tf message TODO: maybe an new unique topic would be more useful here?
                    self.pub_tf.sendTransform(trans,
                                              rot,
                                              rospy.Time.now(),
                                              new_info.vehicle_name,
                                              'Tag'+str(fixed_frame))

                    #TODO: Test if this output is actually correct
                    # Debugging Output
                    trans_rnd = ['%.2f' % elem for elem in trans]
                    rot_rnd   = ['%.2f' % elem for elem in rot]
                    rospy.loginfo("%s: Translation: %s Orientation: %s in reference to Tag%s",
                                   new_info.vehicle_name, trans_rnd, rot_rnd, fixed_frame)



        #     tag_infos.append(new_info)
        #     # --- end tag info processing
        #
        # new_tag_data = AprilTagsWithInfos()
        # new_tag_data.detections = msg.detections
        # new_tag_data.infos = tag_infos
        # # Publish Message
        # self.pub_postPros.publish(new_tag_data)

if __name__ == '__main__':
    rospy.init_node('AprilLocalLocalization',anonymous=False)
    node = AprilLocalLocalization()
    rospy.spin()
