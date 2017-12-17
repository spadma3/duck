import os, sys, pickle, rospy, cv2, threading, functools
import numpy as np
from qt_gui.plugin import Plugin
from cv_bridge import CvBridge
from python_qt_binding import loadUi
from PyQt5 import QtCore
from PyQt5.QtGui import *
from python_qt_binding.QtWidgets import *
from sensor_msgs.msg import Image
from duckietown_msgs.msg import SourceTargetNodes
from fleet_planning.transformation import PixelAndMapTransformer, MapToGraphTransformer
from fleet_planning.generate_duckietown_map import graph_creator
from fleet_planning.map_drawing import MapDraw
from fleet_planning.message_serialization import InstructionMessageSerializer, LocalizationMessageSerializer
from std_msgs.msg import ByteMultiArray
#from fleet_planning.graph import Graph

class RQTFleetPlanning(Plugin):

    def __init__(self, context):
        super(RQTFleetPlanning, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Fleet-Planning')

        # initializing
        self.image = QPixmap()
        self.request_start_node = ''
        self.request_destination_node = ''
        self.basic_map_image = []
        self._all_living_duckiebots = dict()

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_fleet_planning.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_fleet_planning')

        # Load parameters
        self.map_name = rospy.get_param('/map_name', 'tiles_lab')
        self.script_dir = os.path.dirname(__file__)
        self.super_script_dir = self.script_dir + '/../../src/maps/'
        self.tile_size = rospy.get_param('tile_size',101)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # transformations
        self.duckietown_graph_creator = graph_creator()
        self.duckietown_graph = self.duckietown_graph_creator.build_graph_from_csv(self.super_script_dir, self.map_name)
        self.map_to_graph_transformer = MapToGraphTransformer(self.duckietown_graph)
        self.setImageToMapTransformer()

        # ROS publishers/subscribers
        self.pub = rospy.Publisher('~/customer_requests', SourceTargetNodes, queue_size=1, latch=True)
        self._subscriber_map_graph = rospy.Subscriber('/taxi_central_node/map_graph', Image,
                                                      self.image_callback, queue_size = 1)
        self._subscriber_duckiebot_list = rospy.Subscriber('/taxi/location', ByteMultiArray,
                                                           self._received_duckiebot_update_callback)

        #for drawing stuff
        self.basic_map_image = []
        map_dir = rospy.get_param('/map_dir')
        gui_img_dir = rospy.get_param('/gui_img_dir')
        self.map_drawer = MapDraw(self.duckietown_graph, map_dir, gui_img_dir, self.map_name)

        # event handling
        self._widget.buttonFindPlan.clicked.connect(self.requestPlan)
        self._widget.buttonClear.clicked.connect(self.clearRequest)
        self._widget.label_image.mousePressEvent = self.getPos

    def setImageToMapTransformer(self):
        self.image_to_map_transformer = PixelAndMapTransformer(self.tile_size, self.image.height())

    def getPos(self , event):
        tile_position = self.image_to_map_transformer.image_to_map((event.pos().x(), event.pos().y()))
        graph_node_number = self.map_to_graph_transformer.get_closest_node(tile_position)
        self.drawRequestState(tile_position, graph_node_number)

    def drawRequestState(self, tile_position, graph_node_number):
        tile_x = "{:.2f}".format(tile_position[0])
        tile_y = "{:.2f}".format(tile_position[1])
        if (self.isRequestStartSet() and self.isRequestDestinationSet()):
            pass
        elif (self.isRequestStartSet()):
            self._widget.label_dest_tiles.setText("(" + tile_x + ", " + tile_y + ")" + " #" + graph_node_number)
            self.request_destination_node = graph_node_number

        else:
            self._widget.label_start_tiles.setText("(" + tile_x + ", " + tile_y + ")" + " #" + graph_node_number)
            self.request_start_node = graph_node_number
        self.drawCurrentMap()

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
        print('Requesting plan from ' + self.request_start_node + ' to ' + self.request_destination_node)
        self.pub.publish(SourceTargetNodes(self.request_start_node, self.request_destination_node))

    def clearRequest(self):
        self.request_start_node = ""
        self.request_destination_node = ""
        self._widget.label_start_tiles.setText("")
        self._widget.label_dest_tiles.setText("")
        self.drawCurrentMap()

    def shutdown_plugin(self):
        self.pub.unregister()
        self._subscriber_map_graph.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def drawCurrentMap(self):
        #draw start, end, path, duckie on basic_map_image
        basic_map_for_drawing = np.copy(self.basic_map_image)
        if (self.isRequestStartSet()):
            self.map_drawer.draw_icons(basic_map_for_drawing, "customer", self.request_start_node, 1)
        if (self.isRequestDestinationSet()):
            self.map_drawer.draw_icons(basic_map_for_drawing, "target", self.request_destination_node, 1)
        #todo: logic for drawing a plan
        #todo: draw the selected duckiebot
        #convert the drawing to QPixmap for display
        cvImg = cv2.cvtColor(basic_map_for_drawing, cv2.COLOR_BGR2RGB)
        height, width, channel = cvImg.shape
        bytesPerLine = 3 * width
        q_img_tmp = QImage(cvImg.data, width, height, bytesPerLine, QImage.Format_RGB888)
        self.image = QPixmap(q_img_tmp)
        #show it on the GUI
        self._widget.label_image.setGeometry(QtCore.QRect(10, 10, self.image.width(), self.image.height())) #(x, y, width, height)
        self._widget.label_image.setPixmap(self.image)

    def image_callback(self, ros_data):
        bridge = CvBridge()
        self.basic_map_image = bridge.imgmsg_to_cv2(ros_data, "bgr8")
        self.drawCurrentMap()
        self.setImageToMapTransformer()

    def _received_duckiebot_update_callback(self, message):
        duckiebot_name, node, route = LocalizationMessageSerializer.deserialize("".join(map(chr, message.data)))
        #this adds the duckie if it wasn't in there before, otherwise it updates its location
        self._all_living_duckiebots.update({duckiebot_name: node})
        #todo: update the combo_box_items
