#!/usr/bin/env python

import rospy, os, cv2
import numpy as np
from fleet_planning.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from fleet_planning.srv import *
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator
from fleet_planning.transformation import Transformer

class graph_search_server():
    def __init__(self):
        print 'Graph Search Service Started'

        # Input: csv file
        self.map_name = rospy.get_param('/map_name')

        # Loading paths
        self.script_dir = os.path.dirname(__file__)
        self.map_path = self.script_dir + '/maps/' + self.map_name
        self.map_img_path = self.map_path + '_map'
        self.tiles_dir = os.path.abspath(
            self.script_dir + '../../../../30-localization-and-planning/duckietown_description/urdf/meshes/tiles/')
        self.customer_icon_path = os.path.abspath(self.script_dir + '../gui_images/customer_duckie.jpg')

        # build and init graphs
        gc = graph_creator()
        self.duckietown_graph = gc.build_graph_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)
    
        print "Map loaded successfully!\n"

        self.image_pub = rospy.Publisher("~map_graph",Image, queue_size = 1, latch=True)
        self.bridge = CvBridge()

        # prepare and send graph image through publisher
        self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name = self.map_name)

        mc = MapImageCreator(self.tiles_dir)
        self.map_img = mc.build_map_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)

        # image used to store all start, customer and target icons at their positions
        #self.icon_image = np.zeros((self.map_img.shape[1], self.map_img.shape[0], 3), dtype = np.uint8)
        self.customer_icon = cv2.resize(cv2.imread(self.customer_icon_path), (30, 30, 3))
        
        overlay = self.prepImage()
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

    def handle_graph_search(self, req):
        """takes request, calculates path and creates corresponding graph image. returns path"""
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
    
    def draw_icons(self, map_image, trips ):
        """
        Draw start, customer and target icons next to each 
        corresponding graph node along with the respective name 
        of the duckiebot. 
        Input:
            - trips: list of trips, where each trip is a list containing
                     [start location, customer location, target location]
        
        Returns:
            - opencv image with the icons at the correct positions
        """
        #print "Size of map: ", self.map_image.shape

        # loop through all trips currently in existence. For each trip,
        # draw the start, customer and target icons next to the corresponding 
        # label of the graph node. 
        tf = Transformer(80, self.map_img.shape[1] / 80)  # TODO: better way to get the map dimensions?
        for trip in trips:
            print "drawing trip's icons..."
            
            x_start = 50
            x_end = x_start + self.customer_icon.shape[0]
            y_start = 50
            y_end = y_start + self.customer_icon.shape[1]

        map_image[x_start:x_end, y_start:y_end, :] = self.customer_icon

            # self.icon_image = 
        for node in self.duckietown_graph._nodes:
            node_location = np.round(self.duckietown_graph.get_node_pos(node) * 80)  # tile_length = 80 pixels
            self.icon_image[node_location[0], node_location[1], 0] = 255  # R
            self.icon_image[node_location[0], node_location[1], 1] = 0  # G
            self.icon_image[node_location[0], node_location[1], 2] = 0  # B
        # self.duckietown_graph.graph.get_node_posi
         # mask = cv2.cvtColor(self.icon_image, cv2.COLOR_BGR2GRAY)
        # ret, mask = cv2.threshold(mask, 10, 255, cv2.THRESH_BINARY)
        # mask_inv = cv2.bitwise_not(mask)
        # final_image_bg = cv2.bitwise_and(overlay, overlay, mask = mask_inv)
        # final_image_fg = cv2.bitwise_and(self.icon_image, self.icon_image, mask = mask)
        # final_image = cv2.add(final_image_bg, final_image_fg)

        # add all customer logos where desired
        

        # overlay = cv2.addWeighted(self.icon_image, 1, overlay, 0.5, 0)
        return map_image

    def publishImage(self, req, path):
        if path:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=path.edges(), map_name=self.map_name,
                                       highlight_nodes=[req.source_node, req.target_node])
        else:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name=self.map_name)

        overlay = self.prepImage()
        overlay = self.draw_icons(overlay, trips = [2, 2, 2])
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(overlay, "bgr8"))

    def prepImage(self):
        """takes the graph image and map image and overlays them"""
        # TODO: add the icon image and merge it as well
        inverted_graph_img = 255 - self.graph_image
        # bring to same size
        inverted_graph_img = cv2.resize(inverted_graph_img, (self.map_img.shape[1], self.map_img.shape[0]))

        # overlay images
        overlay = cv2.addWeighted(inverted_graph_img, 1, self.map_img, 0.5, 0)

        # make the image bright enough for display again
        hsv = cv2.cvtColor(overlay, cv2.COLOR_BGR2HSV)
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
