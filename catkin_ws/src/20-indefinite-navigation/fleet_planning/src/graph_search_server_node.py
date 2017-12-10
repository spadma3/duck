#!/usr/bin/env python

import rospy, os, cv2
import numpy as np
from fleet_planning.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from fleet_planning.srv import *
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator
from fleet_planning.transformation import Transformer
from fleet_planning.location_to_graph_mapping import IntersectionMapper

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
        self.customer_icon_path = os.path.abspath(self.script_dir + '/../include/gui_images/customer_duckie.jpg')
        self.start_icon_path = os.path.abspath(self.script_dir + '/../include/gui_images/duckie.jpg')
        self.target_icon_path = os.path.abspath(self.script_dir + '/../include/gui_images/location-icon.png')

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
        self.tile_length = mc.tile_length
        self.map_img = mc.build_map_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)

        # image used to store all start, customer and target icons at their positions
        #self.icon_image = np.zeros((self.map_img.shape[1], self.map_img.shape[0], 3), dtype = np.uint8)
        print(self.customer_icon_path)
        self.customer_icon = cv2.resize(cv2.imread(self.customer_icon_path), (30, 30))
        self.start_icon = cv2.resize(cv2.imread(self.start_icon_path), (30, 30))
        self.target_icon = cv2.resize(cv2.imread(self.target_icon_path), (30, 30))
        
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
    
    def graph_node_to_image_location(self, graph, node):
        """
        Convert a graph node number to a 2d image pixel location
        """
        print "graph.node_positions", graph.node_positions
        return graph.node_positions[node]

    def draw_icons(self, map_image, icon_type, location ):
        """
        Draw start, customer and target icons next to each 
        corresponding graph node along with the respective name 
        of the duckiebot. 
        Input:
            - map_image: the base map image onto which to draw the icons
            - icon_type: string, either customer, start or target
            - location: where to draw the icon, as a graph node number
        
        Returns:
            - opencv image with the icons at the correct positions
        """
        #print "Size of map: ", self.map_image.shape

        # loop through all trips currently in existence. For each trip,
        # draw the start, customer and target icons next to the corresponding 
        # label of the graph node. 
        print "self.map_img.shape: ", self.map_img.shape
        transf = Transformer(self.tile_length, self.map_img.shape[0] / self.tile_length)  # TODO: better way to get the map dimensions?
        if icon_type == "customer":
            icon = self.customer_icon
        elif icon_type == "start":
            icon = self.start_icon
        elif icon_type == "target":
            icon = self.target_icon
        else: 
            print "invalid icon type"
            # return

        # convert graph number to 2D image pixel coords
        point = self.graph_node_to_image_location(graph = self.duckietown_graph, node = location)
        print "Point received is: ", point
        point = transf.map_to_image(point)
        print "Point received is: ", point
        x_start = point[1]
        x_end = x_start + icon.shape[0]
        y_start = point[0]  
        y_end = y_start + icon.shape[1]
        map_image[x_start:x_end, y_start:y_end, :] = icon

        return map_image

    def publishImage(self, req, path):
        if path:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=path.edges(), map_name=self.map_name,
                                       highlight_nodes=[req.source_node, req.target_node])
        else:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name=self.map_name)

        print req.source_node, req.target_node
        # TODO: either pass the name of the nodes here and use the  self.duckietown_graph.get_node_oos(node) function
        # or figure out some other way to get the location. 
        overlay = self.prepImage()
        print "req: ", req.source_node, req.target_node
        # draw request if initialized, i.e. nonzero
        if req.target_node != '0':
            overlay = self.draw_icons(overlay, "start", location = req.source_node)
            overlay = self.draw_icons(overlay, "target", location = req.target_node) 
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
