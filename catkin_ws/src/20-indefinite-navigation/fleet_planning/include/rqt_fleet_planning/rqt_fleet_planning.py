import os, sys, pickle, rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget
from duckietown_msgs.msg import SourceTargetNodes
from fleet_planning.transformation import Transformer
#path_dir = os.path.dirname(__file__) + '/../../scripts/'
#sys.path.append(path_dir)
from fleet_planning.generate_duckietown_map import graph_creator, MapImageCreator

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

        # ROS stuff
        self.veh = rospy.get_param('/veh')
        self.topic_name = '/' + self.veh + '/actions_dispatcher_node/plan_request'
        self.pub = rospy.Publisher(self.topic_name,SourceTargetNodes, queue_size=1, latch=True)
        self._widget.buttonFindPlan.clicked.connect(self.requestPlan)
        self._widget.buttonClear.clicked.connect(self.clearRequest)

        #loading a map image
        image_path = os.path.abspath(self.super_script_dir + '/maps/' + self.map_name + '_map.png')
        tiles_path = os.path.abspath(self.super_script_dir +
                                              '../../../../30-localization-and-planning/duckietown_description/urdf/meshes/tiles/')

        image = QtGui.QPixmap(image_path)
        self._widget.label_image.setGeometry(QtCore.QRect(10, 10, image.width(), image.height())) #(x, y, width, height)
        self._widget.label_image.setPixmap(image)
        self._widget.label_image.mousePressEvent = self.getPos
        self.transformer = Transformer(self.tile_size,image.height())

    def getPos(self , event):
        tile_position = self.transformer.image_to_map((event.pos().x(),event.pos().y()))
        self.drawRequestState(tile_position)
     # todo: change to node number

    def drawRequestState(self, tile_position):
        if (self.isRequestStartSet() and self.isRequestDestinationSet()):
            pass
        elif (self.isRequestStartSet()):
            self.request_destination_node = "(" + str(tile_position[0]) + ", " + str(tile_position[1]) + ")"
            # todo: actually draw start and end into the image
        else:
            self.request_start_node = "(" + str(tile_position[0]) + ", " + str(tile_position[1]) + ")"
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
        #self.pub.unregister()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
