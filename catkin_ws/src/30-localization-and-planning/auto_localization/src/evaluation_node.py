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
        self.tag_transformation = dict()

    def callback(self,msg):
        tfs = msg.poses
        tag_dict = {}

        ground_truth = {
        399:
        np.array([[ 1.        ,  0.        ,  0.        ,  0.        ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        373:
        np.array([[ 1.        ,  0.        ,  0.        ,  0.581     ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        397:
        np.array([[ 1.        ,  0.        ,  0.        ,  1.178     ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        375:
        np.array([[ 1.        ,  0.        ,  0.        ,  1.749     ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        376:
        np.array([[ 1.        ,  0.        ,  0.        ,  2.340     ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        377:
        np.array([[ 1.        ,  0.        ,  0.        ,  2.916     ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        340:
        np.array([[ 1.        ,  0.        ,  0.        ,  3.517     ],
               [ 0.        ,  1.        ,  0.        ,  0.        ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        398:
        np.array([[ 1.        ,  0.        ,  0.        ,  0.306     ],
               [ 0.        ,  1.        ,  0.        ,  0.676     ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        396:
        np.array([[ 1.        ,  0.        ,  0.        ,  0.885     ],
               [ 0.        ,  1.        ,  0.        ,  0.676     ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        388:
        np.array([[ 1.        ,  0.        ,  0.        ,  1.469     ],
               [ 0.        ,  1.        ,  0.        ,  0.676     ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        374:
        np.array([[ 1.        ,  0.        ,  0.        ,  2.052     ],
               [ 0.        ,  1.        ,  0.        ,  0.676     ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        389:
        np.array([[ 1.        ,  0.        ,  0.        ,  2.634     ],
               [ 0.        ,  1.        ,  0.        ,  0.676     ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]]),
        372:
        np.array([[ 1.        ,  0.        ,  0.        ,  3.221     ],
               [ 0.        ,  1.        ,  0.        ,  0.676     ],
               [ 0.        ,  0.        ,  1.        ,  0.        ],
               [ 0.        ,  0.        ,  0.        ,  1.        ]])
        }

        relative_data = {}




        for tf_node in tfs:
            print tf_node.host
            print tf_node.frame_id
            print tf_node.bot_id
            trans, rot = gposf.get_trans_rot_from_pose(tf_node.posestamped.pose)
            print trans[0], trans[1]

            if not tag_dict.has_key((tf_node.host, tf_node.frame_id,tf_node.bot_id)):
                tag_dict[tf_node.host, tf_node.frame_id, tf_node.bot_id] = [[trans],[rot]]
            else:
                tag_dict[tf_node.host, tf_node.frame_id, tf_node.bot_id][0].append(trans)
                tag_dict[tf_node.host, tf_node.frame_id, tf_node.bot_id][1].append(rot)


        x = np.array([1,2,3,4,5,6])
        xtics = ['x','y', 'z', 'pitch', 'yaw', 'roll']

        for key in tag_dict:
            # print ground_truth[key[1]]
            a =  ground_truth[key[2]] - ground_truth[key[1]]
            # print a
            # print a[0][3]
            relative_data[key[1:2]] = [a[0][3], a[1][3]]


            fig, ax = plt.subplots()
            plt.xticks(x, xtics)
            ax.set_title(key[0]+", Child frame: "+str(key[1])+", Parent frame: "+str(key[2]))
            plt.ylabel("Absolute Error of Translation [m]", color='b')
            ax.tick_params('y', colors='b')


            ax2 = ax.twinx()
            ax2.set_ylabel('Absolute Error of Orientation []', color='m')
            ax2.tick_params('y', colors='m')


            # plt.xlabel("y")
            # ax.plot(2,relative_data[key[1:2]][1], 'x')
            # ax.plot(1,relative_data[key[1:2]][0], 'x')
            for trans   in tag_dict[key][0]:

                ax.plot(1, trans[0]-relative_data[key[1:2]][0], 'bx')
                ax.plot(2, trans[1]-relative_data[key[1:2]][1], 'bx')
                ax.plot(3, trans[2], 'bx')
                print trans[0], trans[1]

            for rot     in tag_dict[key][1]:
                a,b,c = tr.euler_from_quaternion(rot)
                ax2.plot(4, a, 'mo')
                ax2.plot(5, b, 'mo')
                ax2.plot(6, c, 'mo')





        plt.show()


        # print tag_dict

        #     trans,rot = gposf.get_trans_rot_from_pose(tf_node.posestamped.pose)
        #     if not self.tag_transformation.has_key(tf_node.host):
        #         print "Hello"
        #         # tag_dict[tf_node.host] = []
        #         self.tag_transformation[tf_node.host] = []
        #         print self.tag_transformation
        #
        #     if not tf_node.frame_id in self.tag_transformation[tf_node.host]:
        #
        #
        #         # tag_dict[tf_node.host].append(tf_node.frame_id)
        #         self.tag_transformation[tf_node.host].append(tf_node.frame_id)
        #
        #     # Here we create transformation link with python dictionary between tag and tag
        #     # For convenient, we reuse RemapPose message data type here
        #     # frame_id = child frame, bot_id = parent frame
        #     # self.tag_transformation[child_frame][parent_frame] = [[trans1,trans2],[rot1,rot2]]
        #     if not tf_node.bot_id in self.tag_transformation[tf_node.host][tf_node.frame_id]:
        #         # tag_dict[tf_node.host][tf_node.frame_id].append(tf_node.bot_id)
        #         self.tag_transformation[tf_node.host][tf_node.frame_id].append(tf_node.bot_id)
        #         self.tag_transformation[tf_node.host][tf_node.frame_id][tf_node.bot_id] = [[trans],[rot]]
        #     else:
        #         self.tag_transformation[tf_node.host][tf_node.frame_id][tf_node.bot_id][0].append(trans)
        #         self.tag_transformation[tf_node.host][tf_node.frame_id][tf_node.bot_id][1].append(rot)
        #
        # plot_list = {}
        #
        #
        # for wt in tag_dict:
        #     for frame1 in tag_dict[wt]:
        #         tag_graph.add_node(frame1)
        #         # print tag_graph.nodes
        #         # sys.stdout.write('\r%s' % (tag_graph.nodes))
        #         # sys.stdout.flush()
        #         for frame2 in tag_dict[wt][frame1]:
        #
        #             fig, ax = plt.subplots()
        #             ax.set_title(str(wt)+"Child frame: "+str(frame1)+"Parent frame: "+str(frame2))
        #             ax.plot(self.tag_transformation[wt][frame1][frame2][0][0], self.tag_transformation[wt][frame1][frame2][0][1], 'o')
        #             plt.xlabel("x")
        #             plt.xlabel("y")
        #             plt.show()
        #
        #
        #
        #
        #             tag_graph.add_node(frame2)
        #             # sys.stdout.write('\r%s' % (tag_graph.nodes))
        #             # sys.stdout.flush()
        #             # if len(self.tag_transformation[frame1][frame2][0]) > 1:
        #             trans_mean  =   np.mean(self.tag_transformation[wt][frame1][frame2][0],axis=0)
        #             trans_err   =   np.linalg.norm(np.std(self.tag_transformation[wt][frame1][frame2][0],axis=0))
        #             # print size(self.tag_transformation[frame1][frame2][1])
        #             Q = np.asarray(self.tag_transformation[wt][frame1][frame2][1])
        #             rot_mean    =   avq.averageQuaternions(Q)
        #             # rot_mean    =   np.mean(self.tag_transformation[frame1][frame2][1],axis=0)
        #             rot_err   =   np.linalg.norm(np.std(self.tag_transformation[wt][frame1][frame2][1],axis=0))
        #
        #             # print np.linalg.norm(trans_mean)
        #
        #             # if trans_err > 0.05  or rot_err > 0.6:
        #             #     print "High STD in data of relative pose of: ", frame1, frame2, "\nErrors: ", trans_err, rot_err, np.std(self.tag_transformation[frame1][frame2][0],axis=0), np.std(self.tag_transformation[frame1][frame2][1],axis=0)
        #             # else:
        #             #     self.tag_transformation[wt][frame1][frame2] =[trans_mean.tolist(),rot_mean.tolist()]
        #             #     tag_graph.add_edge(frame1,frame2,np.linalg.norm(trans_mean))
        #             print "Watchtower, TagA, TagB: ", wt, frame1, frame2
### ------------------- ------- MAIN -------------------------------#####

if __name__ == '__main__':
    rospy.init_node('evaluation_node',anonymous=False, disable_signals=True)
    node = evaluation_node()
    rospy.spin()
