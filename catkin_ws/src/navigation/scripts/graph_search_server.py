#!/usr/bin/env python

from navigation.srv import *
import rospy
import sys
import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import pickle
from graph import Graph
from graph_search import GraphSearchProblem

duckietown_graph = 0
draw_solution = True
image_pub = 0
bridge = 0


def handle_graph_search(req):
    global duckietown_graph	
    global draw_solution
    global image_pub
    global bridge
	
	# Checking is nodes exists
    if (req.source_node not in duckietown_graph) or (req.target_node not in duckietown_graph):
        print "Source or target node do not exist."
        return GraphSearchResponse([])

    # Running A*
    duckietown_problem = GraphSearchProblem(duckietown_graph, req.source_node, req.target_node)
    path = duckietown_problem.astar_search()

    # Draw solution
    # TODO: drawing does not work for more than 1 client request
    if path and draw_solution:
        save_name = 'duckietown'
        script_dir = os.path.dirname(__file__)
        map_path = script_dir + '/maps/' + save_name + '.png'
        duckietown_graph.draw(highlight_edges=path.edges(), map_name = save_name)
        cv_image = cv2.imread(map_path, cv2.CV_LOAD_IMAGE_COLOR)
        try:
            image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    return GraphSearchResponse(path.actions)

def graph_search_server():
    s = rospy.Service('graph_search', GraphSearch, handle_graph_search)
    rospy.spin()

if __name__ == "__main__":	
    print 'Graph Search Service Started'

    # Inputs
    map_name = rospy.get_param('map_name', 'duckietown_map.pkl')
    draw_solution = rospy.get_param('draw_solution', True)

    # Loading map
    script_dir = os.path.dirname(__file__)
    map_path = script_dir + '/maps/' + map_name
    try:
	    file2 = open(map_path, 'r')
    except IOError:
	    print "Couldn't find your map:", map_path, ". Closing program..."
	    sys.exit(0)
	
    map_data = pickle.load(file2)
    file2.close()

    # Create graph
    duckietown_graph = Graph()
    edges = map_data[0]
    node_locations = map_data[1]
    for edge in edges:
	    duckietown_graph.add_edge(edge[0], edge[1], edge[2], edge[3])
	
    duckietown_graph.set_node_positions(node_locations)

    print "Map loaded successfully! Starting server...\n"

    rospy.init_node('graph_search_server')
    image_pub = rospy.Publisher("~map_graph",Image, queue_size = 1)
    bridge = CvBridge()

    graph_search_server()
