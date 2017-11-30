#!/usr/bin/env python

import rospy, sys, os, cv2, pickle
from fleet_planning.graph import Graph
from fleet_planning.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from fleet_planning.srv import *
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator
import numpy as np

class graph_search_server():
    def __init__(self):
        print 'Graph Search Service Started'

        # Input: csv file
        self.map_name = rospy.get_param('/map_name')

        # Loading paths
        self.script_dir = os.path.dirname(__file__)
        self.map_path = self.script_dir + '/maps/' + self.map_name
        self.map_img_path = self.map_path + '_map'
        #todo: make this way more robust
        self.tiles_dir = os.path.abspath(self.script_dir + '../../../../30-localization-and-planning/duckietown_description/urdf/meshes/tiles/')

        gc = graph_creator()
        self.duckietown_graph = gc.build_graph_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)
    
        print "Map loaded successfully!\n"

        self.image_pub = rospy.Publisher("~map_graph",Image, queue_size = 1, latch=True)
        self.bridge = CvBridge()

        # Send graph through publisher
        self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name = self.map_name)
        graph_image = cv2.imread(self.map_path + '.png', cv2.IMREAD_COLOR)

        self.mc = MapImageCreator(self.tiles_dir)
        self.map_img = self.mc.build_map_from_csv(script_dir=self.script_dir, csv_filename=self.map_name, graph_width=w, graph_height=h)

        overlay = self.prepImage(graph_image, self.map_img)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

    def handle_graph_search(self, req):
        # Checking if nodes exists
        if (req.source_node not in self.duckietown_graph) or (req.target_node not in self.duckietown_graph):
            print "Source or target node do not exist."
            self.publishImage(req, [])
            return GraphSearchResponse([])

        # Running A*
        self.duckietown_problem.start = req.source_node
        self.duckietown_problem.goal = req.target_node
        path = self.duckietown_problem.astar_search()

        # Publish graph solution
        self.publishImage(req, path)

        return GraphSearchResponse(path.actions)        

    def publishImage(self, req, path):
        if path:
            self.duckietown_graph.draw(self.script_dir, highlight_edges=path.edges(), map_name=self.map_name, highlight_nodes = [req.source_node, req.target_node])
        else:
            self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name=self.map_name)

        graph_image = cv2.imread(self.map_path + '.png', cv2.IMREAD_COLOR)
        overlay = self.prepImage(graph_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

    def prepImage(self, graph_img):
        inverted_graph_img = 255 - graph_img

        # crop to same size
        inverted_graph_img = inverted_graph_img[:self.map_img.shape[0], :self.map_img.shape[1], :]

        # overlay images
        overlay = cv2.addWeighted(inverted_graph_img, 1, self.map_img, 0.5, 0)

        # some color operations
        hsv = cv2.cvtColor(overlay, cv2.COLOR_BGR2HSV)  # convert it to hsv
        h, s, v = cv2.split(hsv)
        lim = 255 - 60
        v[v > lim] = 255
        v[v <= lim] += 60
        final_hsv = cv2.merge((h, s, v))

        overlay = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return overlay

if __name__ == "__main__":
    rospy.init_node('graph_search_server_node')
    gss = graph_search_server()
    print 'Starting server...\n'
    s = rospy.Service('graph_search', GraphSearch, gss.handle_graph_search)
    rospy.spin()
