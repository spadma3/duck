import os, sys, pickle, rospy, cv2, threading, functools
import numpy as np
from qt_gui.plugin import Plugin
from cv_bridge import CvBridge
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget
from sensor_msgs.msg import Image
from duckietown_msgs.msg import SourceTargetNodes
from fleet_planning.transformation import Transformer
from fleet_planning.generate_duckietown_map import graph_creator
from fleet_planning.graph import Graph
from fleet_planning.location_to_graph_mapping import IntersectionMapper

class RQTFleetPlanning(Plugin):

    def __init__(self, context):
        super(RQTFleetPlanning, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Fleet-Planning')

        # flags for the UI input control states
        self.request_start_node = ""
        self.request_destination_node = ""

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_fleet_planning.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_fleet_planning')

        #Load parameters
        self.map_name = rospy.get_param('/map_name', 'tiles_lab')
        self.script_dir = os.path.dirname(__file__)
        self.super_script_dir = self.script_dir + '/../../src/'
        self.tile_size = rospy.get_param('tile_size',101)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        image_path = os.path.abspath(self.super_script_dir + '/maps/' + self.map_name + '_map.png')
        self.image = QtGui.QPixmap(image_path)
        self._widget.label_image.setPixmap(self.image)
        self._widget.label_image.setGeometry(QtCore.QRect(10, 10, self.image.width(), self.image.height())) #(x, y, width, height)

        # ROS stuff
        self.veh = rospy.get_param('/veh')
        self.topic_name = '/' + self.veh + '/actions_dispatcher_node/plan_request'
        self.pub = rospy.Publisher(self.topic_name,SourceTargetNodes, queue_size=1, latch=True)
        self._widget.buttonFindPlan.clicked.connect(self.requestPlan)
        self._widget.buttonClear.clicked.connect(self.clearRequest)
        self.subscriber = rospy.Subscriber('/espresso/graph_search_server_node/map_graph', Image,
                                      self.image_callback,  queue_size = 1)

        self.duckietown_graph_creator = graph_creator()
        self.duckietown_graph = self.duckietown_graph_creator.build_graph_from_csv(self.super_script_dir, self.map_name)
        rospy.wait_for_message('/espresso/graph_search_server_node/map_graph', Image)
        self.setTransformer()

        #generate the label
        self._widget.label_image.mousePressEvent = self.getPos
        #todo: for some reason this segfaults, fix it!
        #self._widget.label_image.setGeometry(QtCore.QRect(10, 10, self.image.width(), self.image.height())) #(x, y, width, height)
        #self._widget.label_image.setPixmap(self.image)

    def setTransformer(self):
        self.map_to_graph_transformer = IntersectionMapper(self.duckietown_graph_creator, self.duckietown_graph)
        self.image_to_map_transformer = Transformer(self.tile_size, self.image.height())

    def getPos(self , event):
        tile_position = self.image_to_map_transformer.image_to_map((event.pos().x(), event.pos().y()))
        graph_node_number = self.map_to_graph_transformer.get_closest_node(tile_position)
        self.drawRequestState(tile_position, graph_node_number)
     # todo: change to node number

    def drawRequestState(self, tile_position, graph_node_number):
        if (self.isRequestStartSet() and self.isRequestDestinationSet()):
            pass
        elif (self.isRequestStartSet()):
            self.request_destination_node = "(" + str(tile_position[0]) + ", " + str(tile_position[1]) + ")" + " #" + graph_node_number
            # todo: actually draw start and end into the image
        else:
            self.request_start_node = "(" + str(tile_position[0]) + ", " + str(tile_position[1]) + ")" + " #" + graph_node_number
            # todo: actually draw start only

        self._widget.label_start_tiles.setText(self.request_start_node)
        self._widget.label_dest_tiles.setText(self.request_destination_node)

    def isRequestStartSet(self):
        if (self.request_start_node):
            return True
        else:
            return False

    def isRequestDestinationSet(self):
        if (self.request_destination_node):
            return True
        else:
            return False

    def requestPlan(self):
        self.pub.publish(SourceTargetNodes(self.request_start_node, self.request_destination_node))

    def clearRequest(self):
        self.request_start_node = ""
        self.request_destination_node = ""
        self._widget.label_start_tiles.setText(self.request_start_node)
        self._widget.label_dest_tiles.setText(self.request_destination_node)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.pub.unregister()
        self.subscriber.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def image_callback(self, ros_data):
        print('callback entered')
        bridge = CvBridge()
        image_np = bridge.imgmsg_to_cv2(ros_data, "bgr8")
        print(image_np.shape)
        self.image = QtGui.QPixmap(QtGui.QImage(image_np, image_np.shape[0],image_np.shape[1],QtGui.QImage.Format_RGB32))
        self.subscriber.unregister()

if __name__ == '__main__':
     super_script_dir = "/home/nico/duckietown/catkin_ws/src/20-indefinite-navigation/fleet_planning/src"
     map_name = "tiles_lab"
     tile_size = 101
     # all those transformations
     duckietown_graph_creator = graph_creator()
     duckietown_graph = duckietown_graph_creator.build_graph_from_csv(super_script_dir, map_name)
     map_to_graph_transformer = IntersectionMapper(duckietown_graph_creator, duckietown_graph)
     #image_to_map_transformer = Transformer(tile_size, 606)
     tile_position = (1,1)
     graph = Graph()
     graph_node_number = map_to_graph_transformer.get_closest_node(tile_position)
     print(tile_position)
     print(graph_node_number)
