#!/usr/bin/env python

## AIDO localization server side postprocessing
# Author: Chen-Lung Eric Lu, NCTU, eric565648.eed03@g2.nctu.edu.tw

## This script converts relative positions of the localization tool
# into a abslute positions
# w.r.t tile (which is different from previous approach)


## coordinates: Origin in top left corner, theta counterclockwise from x-axis
#  O -----> x
#  |
#  |
#  v  y

## Input:
#      - Map where the localization happens
#               .yaml file
#       - Port where the localization data from the watchtowers arrives
#           - Relative Position Data of Duckiebots to fixed Apriltags
#                  ##### data format TBD

## Output:
#       - Absolute position of Duckiebots from the origin in the top left corner
#
#       Statistics file
#       | time | Bot ID | x | y | theta | camera observed | reference Apriltags


import rospkg
import rospy
# import sys # not needed anymore
import yaml
import numpy as np
from datetime import datetime
import tf
import tf.transformations as tr
import global_pose_functions as gposf
import math

## package for list sorting
##from operator import itemgetter

#add local poses Message (Eric add on 0626)
from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose


# import message format of the fixed tags
# import message format of the relative_positions

TILE_SIZE = 0.624 - 0.021 # in meter

class global_localization(object):
    """ """

    def __init__(self):
        """ """
        self.node_name = 'global_localization_node'

        # Load map file path sys.argv[1]
        # self.map_filename = sys.argv[1]

        # Load map file path with ros param set in launch file
        self.map_filename = rospy.get_param("~map") + ".yaml"
        self.load_map_info()

        # Open Output .csv file
        self.output_file_name = rospy.get_param("~output_file")
        self.output_file = self.init_output_file(self.output_file_name)

        #Add Subscriber (Eric add on 0626)
        self.sub_local_pos = rospy.Subscriber("local_poses", RemapPoseArray, self.local_poses_callback, queue_size=1)

        #Add Publisher
        self.pub_abs_pos = rospy.Publisher("~bot_global_poses", GlobalPoseArray, queue_size=1)

### ------------------ LOCAL POSES CALLBACK FUNCTION --------------------#####
# Listens to Local Poses
# Returns Global Poses <3

    def local_poses_callback(self, locpos_msgs):
        # Message is an Array need to iterate trogh each element
        # The node will publish an array full of poses of robots
        global_poses = GlobalPoseArray()
        has_this_bot_been_detected_already = {}

        for locpos_msg in locpos_msgs.poses:

            # Each element of the array is an GlobalPose()
            g_pose = GlobalPose()

            g_pose.bot_id = locpos_msg.bot_id
            g_pose.reference_tag_id.append(locpos_msg.frame_id)

            # time of global pose should be same as relative pose
            g_pose.header.stamp = locpos_msg.posestamped.header.stamp

            # check if reference tag is in the map
            key = "Tag"+str(locpos_msg.frame_id)
            if key in self.fixed_tags.keys():
                g_pose.pose.x, g_pose.pose.y, g_pose.pose.theta = self.transform_bot_position(locpos_msg)
            else:
                print key, " is not in the data base, skip converting to global pose"
                continue

            # Which watchtowers have seen the duckiebot. take last to charakters of
            # string such that "watchtower02" turns to 02
            g_pose.cam_id.append(int(locpos_msg.host[-2:]))




            # check if this bot has been detected already
            if g_pose.bot_id in has_this_bot_been_detected_already.keys():
                # do something
                has_this_bot_been_detected_already[g_pose.bot_id].append(len(global_poses.poses))
            else:
                has_this_bot_been_detected_already[g_pose.bot_id] = [len(global_poses.poses)]


            global_poses.poses.append(g_pose) # Add the pose to the pose array

            # Dump new global pose information into output file
            # TODO do this with the entire GlobalPoseArray
            new_data = [g_pose.header.stamp, g_pose.bot_id,  g_pose.pose.x, g_pose.pose.y, g_pose.pose.theta, g_pose.cam_id ,g_pose.reference_tag_id]
            self.write_data_to_output_file(new_data)


        # before publishing the global_poses merge the redundant poses
        self.pub_abs_pos.publish(global_poses) # Publish the global poses


### -------------- FIND GLOBAL POSE FROM MULITPLE LOCAL POSES------------#####
# This function takes mulitiple local poses from the same time frame
# and converts it to a single global pose
    def estimate_pose(self, global_poses):



        print "Hello World"





### ------------------ INITIALIZATION FUNCTIONS -------------------------#####

    # reads data from the map file
    def load_map_info(self):
        map_data = yaml.load(file(rospkg.RosPack().get_path('auto_localization')+"/config/"+self.map_filename,'r')) # Need RosPack get_path to find the file path
        print "Loaded map from file", self.map_filename, "\nLoading Fixed Tags.."

        self.fixed_tags = {}
        fixed_tags_data = map_data['fixed_tags']
        for fixed_tag in fixed_tags_data:

            tag_id        = fixed_tag['id']
            #trans_tag_abs = fixed_tag['translation']
            #rot_tag_abs   = fixed_tag['orientation']

            # Save Transformation Matrix of each fixed Tag into a dictionary
            #tag_tf_mat = gposf.create_tf_matrix(trans_tag_abs, rot_tag_abs)

            tag_tf_vec = [fixed_tag['translation'], fixed_tag['orientation'], fixed_tag['tile']]

            # since now we save the transformation directily
            self.fixed_tags['Tag'+str(tag_id)] = tag_tf_vec

            #print "Fixed Tag", tag_id, " at Position: ", trans_tag_abs, " and Rotation: ", rot_tag_abs
            print "Fixed Tag", tag_id, " transformation: ", tag_tf_vec



    # creates and initilizes the output file
    # INPUT:    none
    # OUTPUT:   output_file object
    def init_output_file(self, filename):
        time = "{:%Y%m%d-%H%M%S}".format(datetime.now())
        filename_dates = filename +time + ".csv"
        output_file_name = rospkg.RosPack().get_path('auto_localization') + "/config/" + filename_dates
        print output_file_name
        output_file = open(output_file_name, 'w+')
        output_file.write('time, bot_ID, x, y, theta, camera_id, reference_tag_id\n')
        return output_file



### ---------------------- TRANSFORMATION FUNCTION ----------------------#####

# utility functions are in global_pose_functions.py

    def transform_bot_position(self, local_pose):

        # First step: Proect the pose to 2D surface (w.r.t the local pose)

        bot_x = local_pose.posestamped.pose.position.x
        bot_y = local_pose.posestamped.pose.position.y
        o = local_pose.posestamped.pose.orientation
        qua_mat = [o.x, o.y, o.z, o.w]
        roll, pitch, yaw = tr.euler_from_quaternion(qua_mat)
        bot_theta = yaw

        # Second step: Transform from local pose to global pose

        trans, rots, tile = self.fixed_tags['Tag'+str(local_pose.frame_id)]
        rots = rots * math.pi
        ## Doing 2D pose transformation (What you learn in hight school)
        ## Do rotation first
        bot_theta_after = bot_theta - rots
        bot_x_after = bot_x*math.cos(-1*rots) + bot_y*math.sin(-1*rots)
        bot_y_after = -1*bot_x*math.sin(-1*rots) + bot_y*math.cos(-1*rots)
        ## Then do the transformation of point
        # to local tile orgin + trans
        # then to global origin TILE_SIZE*tile
        bot_x_after = bot_x_after + trans[0]*0.01 + TILE_SIZE*tile[0]
        bot_y_after = bot_y_after + trans[1]*0.01 + TILE_SIZE*tile[1]

        return bot_x_after, bot_y_after, bot_theta_after


### --------------------- USER OUTPUT FUNCTIONS --------------------#####
    # writes new data to the output file
    def write_data_to_output_file(self,new_data):
        for idx in xrange( 0, (len(new_data)-1) ):
            self.output_file.write(str(new_data[idx]) + ', ')
        self.output_file.write(str(new_data[len(new_data)-1]) + '\n')





### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('global_localization',anonymous=False)
    node = global_localization()
    rospy.spin()



    # save statistics
    # output_file.close()
