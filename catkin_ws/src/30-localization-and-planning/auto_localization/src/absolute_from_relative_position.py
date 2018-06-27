#!/usr/bin/env python

## AIDO localization server side postprocessing
# Author: Josefine Quack, ETHZ, jquack@ethz.ch

## This script converts relative positions of the localization tool
# into a abslute positions
# how to launch this program:
# $ source environment.sh
# $ python absolute_from_relative_position.py 'path/to/map.yaml' 'port'

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
import sys
import yaml
import numpy as np
from datetime import datetime
import tf
import tf.transformations as tr

#add local poses Message (Eric add on 0626)
from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose


# import message format of the fixed tags
# import message format of the relative_positions



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
        self.output_file = self.init_output_file()

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

            # TODO: handle if bot is detected multiple times
            # if g_pose.bot_id in has_this_bot_been_detected_already.keys()
            g_pose.bot_id = locpos_msg.bot_id

            g_pose.reference_tag_id.append(locpos_msg.frame_id)

            # time of global pose should be same as relative pose
            g_pose.header.stamp = locpos_msg.posestamped.header.stamp
            g_pose.pose.x, g_pose.pose.y, g_pose.pose.theta = self.transform_bot_position(locpos_msg)


            # Which watchtowers have seen the duckiebot. take last to charakters of
            # string such that "watchtower02" turns to 02
            #print locpos_msg.host[-2:]
            #print int(locpos_msg.host[-2:])

            g_pose.cam_id.append(int(locpos_msg.host[-2:]))

            global_poses.poses.append(g_pose) # Add the pose to the pose array

            # Dump new global pose information into output file
            # TODO do this with the entire GlobalPoseArray
            new_data = [g_pose.header.stamp, g_pose.bot_id,  g_pose.pose.x, g_pose.pose.y, g_pose.pose.theta, g_pose.cam_id ,g_pose.reference_tag_id]
            self.write_data_to_output_file(new_data)

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
            trans_tag_abs = fixed_tag['translation']
            rot_tag_abs   = fixed_tag['orientation']

            # Save Transformation Matrix of each fixed Tag into a dictionary
            tag_tf_mat = self.create_tf_matrix(trans_tag_abs, rot_tag_abs)
            self.fixed_tags['Tag'+str(tag_id)] = tag_tf_mat


            print "Fixed Tag", tag_id, " at Position: ", trans_tag_abs, " and Rotation: ", rot_tag_abs



    # creates and initilizes the output file
    # INPUT:    none
    # OUTPUT:   output_file object
    def init_output_file(self):
        path = "/home/jquack/duckietown/statistics/"
        time = "{:%Y%m%d-%H%M%S}".format(datetime.now())
        output_file_name = path + 'localization-statistics-' + time + '.csv'
        print output_file_name
        output_file = open(output_file_name, 'w+')
        output_file.write('time, bot_ID, x, y, theta, camera_id, reference_tag_id\n')
        return output_file





### ---------------------- TRANSFORMATION FUNCTIONS ----------------------#####
    def create_tf_matrix(self, trans, rot):
        # numpy arrays to 4x4 transform matrix
        trans_mat = tr.translation_matrix(trans)
        rot_mat = tr.quaternion_matrix(rot)
        # create a 4x4 matrix
        transformation_matrix = np.dot(trans_mat, rot_mat)
        return transformation_matrix



    # This function projects the 3D position onto the 2D plane
    def project_position_to_2D_plane(self,mat_bot_abs):
        # go back to quaternion and 3x1 array
        trans_bot_abs = tf.transformations.translation_from_matrix(mat_bot_abs)
        x = trans_bot_abs[0]
        y = trans_bot_abs[1]

        rot_bot_abs = tf.transformations.quaternion_from_matrix(mat_bot_abs)

        # returns theta in radians
        alpha,beta, theta = tr.euler_from_matrix(mat_bot_abs, 'rxyz')

        return x,y,theta


    # returns the absolute position of the bot in reference to the origin
    def absolute_from_relative_position(self, mat_bot_tag, mat_tag_abs):
        # https://answers.ros.org/question/215656/how-to-transform-a-pose/

        # TODO: verify order of arguments
        mat_bot_abs = tr.concatenate_matrices(mat_tag_abs, mat_bot_tag)

        return mat_bot_abs


    def transform_bot_position(self, local_pose):
        # trans: xyz translation
        # rot: xyzw quaternion
        # mat: 4x4 transformation matrix
        position = local_pose.posestamped.pose.position
        orientation = local_pose.posestamped.pose.orientation
        trans_bot_tag = [position.x, position.y, position.z]
        rot_bot_tag = [orientation.x, orientation.y, orientation.z, orientation.w]


        # TODO: robostify in case fixed Tag is detected which is not in the database
        #       raise exception or error
        mat_tag_abs = self.fixed_tags["Tag"+str(local_pose.frame_id)]
        mat_bot_tag = self.create_tf_matrix(trans_bot_tag,rot_bot_tag)

        # absolute position of the bot
        mat_bot_abs = self.absolute_from_relative_position(mat_bot_tag, mat_tag_abs)

        # projected Position
        x,y,theta = self.project_position_to_2D_plane(mat_bot_abs)


        return x, y, theta



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
