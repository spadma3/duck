import os
import rospy
import cv2
from cv_bridge import CvBridge
from fleet_planning.graph_search import GraphSearchProblem
from sensor_msgs.msg import Image
from fleet_planning.srv import *
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator
from fleet_planning.transformation import PixelAndMapTransformer


class MapDraw():
    """
    Used to generate the map from a csv file, draw the graph on top
    of that and draw the icons for each duckiebot.
    TODO(ben): add a counter of number of icons at each node and make sure
          to draw overlapping icons next to each other.
    """

    def __init__(self, duckietown_graph, map_dir, gui_img_dir, map_name):
        print 'mapDraw initializing...'

        # Input: csv file
        self.map_name = map_name

        # Loading paths
        self.map_path = map_dir + self.map_name
        self.map_img_path = self.map_path + '_map'
        self.tiles_dir = os.path.abspath(
            map_dir + '/../../../../30-localization-and-planning/duckietown_description/urdf/meshes/tiles/')
        self.customer_icon_path = os.path.join(gui_img_dir, 'customer_duckie.jpg')
        self.start_icon_path = os.path.join(gui_img_dir, 'duckie.jpg')
        self.target_icon_path = os.path.join(gui_img_dir, 'location-icon.png')
        self.script_dir = os.path.dirname(__file__)

        # build and init graphs
        self.duckietown_graph = duckietown_graph  # gc.build_graph_from_csv(script_dir=self.script_dir, csv_filename=self.map_name)
        self.duckietown_problem = GraphSearchProblem(self.duckietown_graph, None, None)  # GraphSearchProblem(self.duckietown_graph, None, None)

        self.bridge = CvBridge()
        # prepare and send graph image through publisher
        self.graph_image = self.duckietown_graph.draw(map_dir=map_dir, highlight_edges=None, map_name=self.map_name)

        mc = MapImageCreator(self.tiles_dir)
        self.tile_length = mc.tile_length
        self.map_img = mc.build_map_from_csv(map_dir=map_dir, csv_filename=self.map_name)

        # keep track of how many icons are being drawn at each node
        self.num_duckiebots_per_node = {node: 0 for node in self.duckietown_graph._nodes}

        # image used to store all start, customer and target icons at their positions
        self.customer_icon = cv2.resize(cv2.imread(self.customer_icon_path), (30, 30))
        self.start_icon = cv2.resize(cv2.imread(self.start_icon_path), (30, 30))
        self.target_icon = cv2.resize(cv2.imread(self.target_icon_path), (30, 30))

        print "Map loaded successfully!\n"

    def graph_node_to_image_location(self, graph, node):
        """
        Convert a graph node number to a 2d image pixel location
        """
        return graph.node_positions[str(node)]

    def draw_icons(self, map_image, icon_type, location, icon_number):
        """
        Draw start, customer and target icons next to each
        corresponding graph node along with the respective name
        of the duckiebot.
        :param map_image: the base map image onto which to draw the icons
        :param icon_type: string, either customer, start or target
        :param location: where to draw the icon, as a graph node number
        :param icon_number: keeps track of how many icons have already
                            been drawn at this location; adjust position
                            accordingly

        :return opencv image with the icons at the correct positions
        """
        # loop through all trips currently in existence. For each trip,
        # draw the start, customer and target icons next to the corresponding
        # label of the graph node.
        transf = PixelAndMapTransformer(self.tile_length, self.map_img.shape[
            0] / self.tile_length)  # TODO: better way to get the map dimensions?
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
        point = self.graph_node_to_image_location(graph=self.duckietown_graph, node=location)
        point = transf.map_to_image(point)
        x_start = point[1]
        x_end = x_start + icon.shape[0]
        y_start = point[0] + (icon_number - 1) * (
        icon.shape[1] + 5)  # TODO: check to make sure icons aren't outside of image boundaries
        y_end = y_start + icon.shape[1]
        map_image[x_start:x_end, y_start:y_end, :] = icon

        return map_image

    def publishImage(self, req, path):
        if path:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=path.edges(),
                                                          map_name=self.map_name,
                                                          highlight_nodes=[req.source_node, req.target_node])
        else:
            self.graph_image = self.duckietown_graph.draw(self.script_dir, highlight_edges=None, map_name=self.map_name)

        # TODO: either pass the name of the nodes here and use the  self.duckietown_graph.get_node_oos(node) function
        # or figure out some other way to get the location.
        overlay = self.prepImage()

        # draw request if initialized, i.e. nonzero
        if req.target_node != '0':
            overlay = self.draw_icons(overlay, "start", location=req.source_node, icon_number=1)
            overlay = self.draw_icons(overlay, "target", location=req.target_node, icon_number=1)
        return self.bridge.cv2_to_imgmsg(overlay, "bgr8")

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

    def drawMap(self, duckiebots):
        """
        New function to draw map independent of GUI calls. Draw all duckiebots
        and their customers, if they have any.
        Input:
            - duckiebots: all duckiebots that should be drawn
        """
        # TODO: figure out best way to visualize duckiebot with customer
        overlay = self.prepImage()
        for name, bot in duckiebots.iteritems():
            self.num_duckiebots_per_node[str(bot._last_known_location)] += 1
            #print "Node ", bot._last_known_location, " visited ", self.num_duckiebots_per_node[
            #    str(bot._last_known_location)], " times."
            overlay = self.draw_icons(overlay, "start", location=bot._last_known_location,
                                      icon_number=self.num_duckiebots_per_node[str(bot._last_known_location)])
            if bot._customer_request:
                overlay = self.draw_icons(overlay, "customer",
                                          location=bot._customer_request.start_location, icon_number=1)  # TODO(ben): figure out an unambiguous set of icons and assign the correct ones
                overlay = self.draw_icons(overlay, "target", location=bot._customer_request.target_location, icon_number=1)

                # set num_duckiebots_per_node back to zero
        for node, num in self.num_duckiebots_per_node.iteritems():
            self.num_duckiebots_per_node[node] = 0
            #print "Node ", node, " set to zero.", self.num_duckiebots_per_node[node]

        return self.bridge.cv2_to_imgmsg(overlay, "bgr8")
