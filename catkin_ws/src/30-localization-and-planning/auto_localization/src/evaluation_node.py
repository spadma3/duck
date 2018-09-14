#!/usr/bin/env python


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
import matplotlib.pyplot as plt
import averageQuaternions as avq

class evaluation_node(object):
    """docstring for ."""
    def __init__(self):
        self.node_name = 'evaluation_node'
        # Subscribe all tfs from subfserver node
        print "evaluation_node started"
        self.sub_tfs = rospy.Subscriber("/duckietown3/system_calibration_gui/complete_local_poses", RemapPoseArray, self.callback, queue_size=1)


    def callback(self,msg):
        tfs = msg.poses
        tag_dict = dict()


        for tf_node in tfs:
            tf_node.host
            trans,rot = gposf.get_trans_rot_from_pose(tf_node.posestamped.pose)
            if not tag_dict.has_key(tf_node.host):
                tag_dict[tf_node.host] = []

                if not tag_dict.has_key(tf_node.frame_id):

                    tag_dict[tf_node.host].append(tf_node.frame_id)
                    self.tag_transformation[tf_node.host][tf_node.frame_id] = dict()

                # Here we create transformation link with python dictionary between tag and tag
                # For convenient, we reuse RemapPose message data type here
                # frame_id = child frame, bot_id = parent frame
                # self.tag_transformation[child_frame][parent_frame] = [[trans1,trans2],[rot1,rot2]]
                if not tf_node.bot_id in tag_dict[tf_node.frame_id]:
                    tag_dict[tf_node.host][tf_node.frame_id].append(tf_node.bot_id)
                    self.tag_transformation[tf_node.host][tf_node.frame_id][tf_node.bot_id] = [[trans],[rot]]
                else:
                    self.tag_transformation[tf_node.host][tf_node.frame_id][tf_node.bot_id][0].append(trans)
                    self.tag_transformation[tf_node.host][tf_node.frame_id][tf_node.bot_id][1].append(rot)

        for wt in tag_dict:
            for frame1 in tag_dict[wt]:
                tag_graph.add_node(frame1)
                # print tag_graph.nodes
                # sys.stdout.write('\r%s' % (tag_graph.nodes))
                # sys.stdout.flush()
                for frame2 in tag_dict[wt][frame1]:
                    tag_graph.add_node(frame2)
                    # sys.stdout.write('\r%s' % (tag_graph.nodes))
                    # sys.stdout.flush()
                    # if len(self.tag_transformation[frame1][frame2][0]) > 1:
                    trans_mean  =   np.mean(self.tag_transformation[wt][frame1][frame2][0],axis=0)
                    trans_err   =   np.linalg.norm(np.std(self.tag_transformation[wt][frame1][frame2][0],axis=0))
                    # print size(self.tag_transformation[frame1][frame2][1])
                    Q = np.asarray(self.tag_transformation[wt][frame1][frame2][1])
                    rot_mean    =   avq.averageQuaternions(Q)
                    # rot_mean    =   np.mean(self.tag_transformation[frame1][frame2][1],axis=0)
                    rot_err   =   np.linalg.norm(np.std(self.tag_transformation[wt][frame1][frame2][1],axis=0))

                    # print np.linalg.norm(trans_mean)

                    # if trans_err > 0.05  or rot_err > 0.6:
                    #     print "High STD in data of relative pose of: ", frame1, frame2, "\nErrors: ", trans_err, rot_err, np.std(self.tag_transformation[frame1][frame2][0],axis=0), np.std(self.tag_transformation[frame1][frame2][1],axis=0)
                    # else:
                    #     self.tag_transformation[wt][frame1][frame2] =[trans_mean.tolist(),rot_mean.tolist()]
                    #     tag_graph.add_edge(frame1,frame2,np.linalg.norm(trans_mean))
                    print "Watchtower, TagA, TagB: ", wt, frame1, frame2
### ------------------- ------- MAIN -------------------------------#####

if __name__ == '__main__':
    rospy.init_node('evaluation_node',anonymous=False, disable_signals=True)
    node = evaluation_node()
    rospy.spin()
