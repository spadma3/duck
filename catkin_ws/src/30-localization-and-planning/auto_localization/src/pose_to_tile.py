#!/usr/bin/env python

#this script maps the positions




import rospkg
import rospy
import sys
import yaml
import numpy as np
from datetime import datetime
import tf
import tf.transformations as tr
import numpy as np
import time
import global_pose_functions as gposf
from duckietown_msgs.msg import RemapPoseArray, RemapPose, GlobalPoseArray, GlobalPose

class tilemapping (object):

    def __init__(self):

        self.node_name = 'tilemapping'

        # # load the map file, notice that the file will be overwritten after calibration
        # self.map_filename = rospy.get_param("~map") + ".yaml"
        # timestr = "{:%Y%m%d-%H%M}".format(datetime.now())
        # self.output_map_filename = rospy.get_param("~output_file") + timestr + ".yaml"
        # self.map_data = self.load_map_info(self.map_filename)

        # Subscribe to global poses
        self.sub_globalposes = rospy.Subscriber("global_poses", GlobalPoseArray, self.callback, queue_size=1)

        #  def callback tilepose
    def callback(self,msg):
        msg_tfs = msg.poses
        print "I received a message"


### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('tilemapping',anonymous=False, disable_signals=True)
    node = tilemapping()
    rospy.spin()
