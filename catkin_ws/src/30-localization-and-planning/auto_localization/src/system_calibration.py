#!/usr/bin/env python

## AIDO localization System Calibration
# Author:  Chen-Lung (Eric) Lu , ETHZ NCTU, eric565648.eed03@g2.nctu.edu.tw
#          Josefine Quack, ETHZ, jquack@ethz.ch

## This script records the positions of all reference Apriltags
# and save them into map file
# The scenerio is that the system will calibrate itself from time to time.

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

class system_calibration(object):

    def __init__(self):

        self.node_name = 'system_calibration'

        # load the map file, notice that the file will be overwritten after calibration
        self.map_filename = rospy.get_param("~map") + ".yaml"
        timestr = "{:%Y%m%d-%H%M}".format(datetime.now())
        self.output_map_filename = rospy.get_param("~output_file") + timestr + ".yaml"
        self.map_data = self.load_map_info(self.map_filename)

        # Subscribe all tfs from subfserver node
        self.sub_tfs = rospy.Subscriber("complete_local_poses", RemapPoseArray, self.callback, queue_size=1)

        # Start Calibration

        #Parameters
        self.start_calibrate = False
        self.wait_for_message = 15 # At least wait 3 secs for tags collection after all watchtower have publish things.
        self.deadline = time.time() + 100000 # set deadline really high at start, will be set to actual value later
        self.ready = False


    # A little recursive function to find the transformation from origin to end_tag
    def from_origin_to_end(self, path):
        # print path
        # If there's no path connect to the tag, saves null.
        if path == None:
            return None
        # self.tag_transformation[child_frame][parent_frame]
        trans = self.tag_transformation[path[0]][path[1]][0]
        rot = self.tag_transformation[path[0]][path[1]][1]
        # print trans, rot
        if len(trans) == 1:
            trans = trans[0]
            rot = rot[0]

        # Compose transformation matrix with translation and angle (in euler)
        transformation_mat = gposf.create_tf_matrix(trans, rot)
        #print len(path)

        if len(path) == 2:
            return transformation_mat
            # print transformation_mat
        else:
            # print "Next transformation", path[1:]
            next_transformation = self.from_origin_to_end(path[1:])

            # print np.dot(transformation_mat, next_transformation)
            return np.dot(transformation_mat, next_transformation)

    #  def sys_calib(self, msg_tfs):
    def callback(self,msg):
        msg_tfs = msg.poses
        print msg_tfs
        self.tag_relationship = self.find_tag_relationship(msg_tfs)
        fixed_tags_data = []
        for tag in self.tag_relationship:
            # Every matrix is the transformation matrix between the tag and origin tag
            # We here decompose the matrix and save only translation and rotation
            tag_data = {}
            tag_data['id'] = tag
            tag_data['transformation'] = self.tag_relationship[tag]
            fixed_tags_data.append(tag_data)


        # Write the transformation relationship to map file
        data = {'tiles':self.map_tiles, 'watchtowers':self.map_watchtowers, 'origin':self.map_origins, 'fixed_tags':fixed_tags_data}
        with open(rospkg.RosPack().get_path('auto_localization')+"/config/"+self.output_map_filename, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

        rospy.loginfo("Calibration Finished!")
        rospy.signal_shutdown("Finished Calibration")

    # find relationship of each tags with origin
    # return a dictionary with tag id as key and translation, rotation as content of each key
    def find_tag_relationship(self, tfs):

        # https://www.python.org/doc/essays/graphs/
        # make all pose to pose relationship into a graph
        # so that we could compute shortest path
        # and get the tag to origin transformation later
        tag_dict = dict() # Save tag node connection graph with a 1D dictionary
        tag_graph = gposf.Graph() # # Save tag node connection graph with own class
        origin = self.map_origins[0]['id'] # a.t.m. we only consider one origin
        tag_graph.add_node(origin)

        self.tag_transformation = dict() # Save tag transformation with a 2D dictionary
        for tf_node in tfs:
            trans,rot = gposf.get_trans_rot_from_pose(tf_node.posestamped.pose)
            if not tag_dict.has_key(tf_node.frame_id):
                tag_dict[tf_node.frame_id] = []
                self.tag_transformation[tf_node.frame_id] = dict()
            # Here we create transformation link with python dictionary between tag and tag
            # For convenient, we reuse RemapPose message data type here
            # frame_id = child frame, bot_id = parent frame
            # self.tag_transformation[child_frame][parent_frame] = [[trans1,trans2],[rot1,rot2]]
            if not tf_node.bot_id in tag_dict[tf_node.frame_id]:
                tag_dict[tf_node.frame_id].append(tf_node.bot_id)
                self.tag_transformation[tf_node.frame_id][tf_node.bot_id] = [[trans],[rot]]
            else:
                self.tag_transformation[tf_node.frame_id][tf_node.bot_id][0].append(trans)
                self.tag_transformation[tf_node.frame_id][tf_node.bot_id][1].append(rot)

        # print "Tag dict:", tag_dict
        # Take mean for redundant information

        # print "This is the information to work with:"


        for frame1 in tag_dict:
            tag_graph.add_node(frame1)
            # print tag_graph.nodes
            # sys.stdout.write('\r%s' % (tag_graph.nodes))
            # sys.stdout.flush()
            for frame2 in tag_dict[frame1]:
                tag_graph.add_node(frame2)
                # sys.stdout.write('\r%s' % (tag_graph.nodes))
                # sys.stdout.flush()
                # if len(self.tag_transformation[frame1][frame2][0]) > 1:
                trans_mean  =   np.mean(self.tag_transformation[frame1][frame2][0],axis=0)
                trans_err   =   np.linalg.norm(np.std(self.tag_transformation[frame1][frame2][0],axis=0))
                # print size(self.tag_transformation[frame1][frame2][1])
                Q = np.asarray(self.tag_transformation[frame1][frame2][1])
                rot_mean    =   avq.averageQuaternions(Q)
                # rot_mean    =   np.mean(self.tag_transformation[frame1][frame2][1],axis=0)
                rot_err   =   np.linalg.norm(np.std(self.tag_transformation[frame1][frame2][1],axis=0))

                # print np.linalg.norm(trans_mean)

                if trans_err > 0.05  or rot_err > 0.6:
                    print "High STD in data of relative pose of: ", frame1, frame2, "\nErrors: ", trans_err, rot_err, np.std(self.tag_transformation[frame1][frame2][0],axis=0), np.std(self.tag_transformation[frame1][frame2][1],axis=0)
                else:
                    self.tag_transformation[frame1][frame2] =[trans_mean.tolist(),rot_mean.tolist()]
                    tag_graph.add_edge(frame1,frame2,np.linalg.norm(trans_mean))
                # print frame1, frame2


        # print tag_graph.nodes
        # print tag_graph.edges
        #
        # # Define a find shortest path function here
        # def find_shortest_path(graph, start, end, path=[]):
        #     path = path + [start]
        #     if start == end:
        #         return path
        #     if not graph.has_key(start):
        #         return None
        #     shortest = None
        #     for node in graph[start]:
        #         if node not in path:
        #             newpath = find_shortest_path(graph, node, end, path)
        #             if newpath:
        #                 if not shortest or len(newpath) < len(shortest):
        #                     shortest = newpath
        #     return shortest



        tag_relationship = dict()
        path_length = {}
        paths = {}
        for tag_node in tag_graph.nodes:
            print "tag_node: ", tag_node
            if tag_node == origin:
                tag_relationship[tag_node] = np.identity(4)
                path_length[tag_node] = 1
            else:
                # path_node = find_shortest_path(tag_dict, origin, tag_node)
                # print tag_graph.nodes
                # print tag_graph.edges.keys()
                path_node = gposf.dijkstra(tag_graph.nodes, tag_graph.edges2, origin, tag_node)
                #print path_node
                #path_node = path_node[0]
                #print path_node
                # path_node = path_node[::-1]
                # print "path_node: ", path_node
                #path_node= path_node[1:]
                print "Path: ", path_node
                if not path_node==None:
                    paths[tag_node]       = path_node
                    path_length[tag_node] = len(path_node)
                else:
                    path_length[tag_node]=0
                tag_relationship[tag_node] = self.from_origin_to_end(path_node)
            print tag_relationship[tag_node]
            # print repr(tag_relationship[tag_node]

        # create a plot from the results
        fig, ax = plt.subplots()
        plt.subplots_adjust(left=0.15, bottom=0.1, right=0.85, top=0.95,
                wspace=None, hspace=None)
        # plt.xticks(x, xtics)
        plt.xlabel("x [m]", color='b', size=16)
        ax.set_title("Result of System Calibration: Global Positions of Mapped Tags ")
        plt.ylabel("y [m]", color='b', size=16)
        # ax.tick_params('y', colors='b')
        ax.margins(0.1)

        for key,value in tag_relationship.iteritems():

            try:
                x = value[0][3]
                y = value[1][3]
                # ax.text(x, y, str(path_length[key]), ha='center', size=20)
                ax.text(x, y, str(key), ha='center', size=14)
            except:
                print "Value for Tag ", key, "is None"

        plt.show()
        save_res = open("/home/duckietown/duckietown/catkin_ws/src/30-localization-and-planning/auto_localization/results/line_test/test_results", 'a+')
        save_res.write("{:%Y%m%d-%H%M}".format(datetime.now()))
        save_res.write("\n"+repr(paths))
        save_res.write("\n"+repr(tag_relationship))





        print tag_relationship


        return tag_relationship

    ## Load Map Data
    def load_map_info(self, filename):

        map_data = yaml.load(file(rospkg.RosPack().get_path('auto_localization')+"/config/"+filename,'r')) # Need RosPack get_path to find the file path
        print "Loaded map from file: ", self.map_filename

        self.map_tiles = map_data['tiles']
        print "\nThis is your map: \n", self.map_tiles

        self.map_watchtowers = map_data['watchtowers']
        print "\nThese watchtowers suppose to work: \n", self.map_watchtowers

        self.map_origins = map_data['origin']
        print "The origins: \n", self.map_origins

        return map_data


### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('system_calibration',anonymous=False, disable_signals=True)
    node = system_calibration()
    rospy.spin()
