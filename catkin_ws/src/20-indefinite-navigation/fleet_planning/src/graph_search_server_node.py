#!/usr/bin/env python

import rospy, os, cv2
from fleet_planning.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from fleet_planning.srv import *
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator


class graph_search_server():
    def __init__(self):
        print 'Graph Search Service Started'

        # Input: csv file
        map_name = rospy.get_param('/map_name')
        map_dir = rospy.get_param('/map_dir')

        # build and init graphs
        gc = graph_creator()
        self.duckietown_graph = gc.build_graph_from_csv(map_dir=map_dir, csv_filename=map_name)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)
        print "Graph loaded successfully!\n"

    def handle_graph_search(self, req):
        """takes request, calculates path and creates corresponding graph image. returns path"""
        # Checking if nodes exists
        if (req.source_node not in self.duckietown_graph) or (req.target_node not in self.duckietown_graph):
            print "Source or target node do not exist."
            return GraphSearchResponse([])

        # Running A*
        self.duckietown_problem.start = req.source_node
        self.duckietown_problem.goal = req.target_node
        path = self.duckietown_problem.astar_search()
        rospy.logwarn('HEE PAATH {}'.format(path.path)0)
        return GraphSearchResponse(path.actions, path.path)


if __name__ == "__main__":
    rospy.init_node('graph_search_server_node')
    gss = graph_search_server()
    print 'Starting server...\n'
    s = rospy.Service('graph_search', GraphSearch, gss.handle_graph_search)
    rospy.spin()
