import os, sys, pickle, rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui
from python_qt_binding.QtWidgets import QWidget
from duckietown_msgs.msg import SourceTargetNodes
from fleet_planning.transformation import Transformer
#path_dir = os.path.dirname(__file__) + '/../../scripts/'
#sys.path.append(path_dir)
from fleet_planning.generate_duckietown_map import graph_creator

class RQTFleetPlanning(Plugin):

    def __init__(self, context):
        super(RQTFleetPlanning, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Fleet-Planning')

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'rqt_fleet_planning.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_fleet_planning')

        #Load parameters
        self.map_name = rospy.get_param('/map_name', 'tiles_lab')
        self.script_dir = os.path.dirname(__file__)
        self.super_script_dir = self.script_dir + '/../../src/'

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.loadComboBoxItems()

        # ROS stuff
        self.veh = rospy.get_param('/veh')
        self.topic_name = '/' + self.veh + '/actions_dispatcher_node/plan_request'
        self.pub = rospy.Publisher(self.topic_name,SourceTargetNodes, queue_size=1, latch=True)
        self._widget.buttonFindPlan.clicked.connect(self.requestPlan)

        #loading a map image
        image_path = os.path.abspath(self.super_script_dir + '/maps/' + self.map_name + '_map.png')
        image = QtGui.QPixmap(image_path)
        self._widget.label_image.setGeometry(QtCore.QRect(10, 10, image.width(), image.height())) #(x, y, width, height)
        self._widget.label_image.setPixmap(image)
        self._widget.label_image.mousePressEvent = self.getPos
        #todo: remove hardcoding
        self.transformer = Transformer(101,6)

    def getPos(self , event):
        self._widget.label_x.setText("Pixel Position x: " + str(event.pos().x()))
        self._widget.label_y.setText("Pixel Position y: " + str(event.pos().y()))
        tile_position = self.transformer.image_to_map((event.pos().x(),event.pos().y()))
        self._widget.label_x_tile.setText("Tile Position x: " + str(tile_position[0]))
        self._widget.label_y_tile.setText("Tile Position y: " + str(tile_position[1]))

    def loadComboBoxItems(self):
        # Loading map
        gc = graph_creator()
        gc.build_graph_from_csv(script_dir=self.super_script_dir, csv_filename=self.map_name)

        node_locations = gc.node_locations
        #comboBoxList = sorted([int(key) for key in node_locations if key[0:4]!='turn'])
        comboBoxList = []
        for key in node_locations:
            if key[0:4] == 'turn':
                continue
            elif int(key) % 2 == 0: # allows only selection of odd numbered nodes
                continue
            comboBoxList += [int(key)]
        comboBoxList = sorted(comboBoxList)
        comboBoxList = [str(key) for key in comboBoxList]
        self._widget.comboBoxDestination.addItems(comboBoxList)
        self._widget.comboBoxStart.addItems(comboBoxList)

    def requestPlan(self):
        start_node = str(self._widget.comboBoxStart.currentText())
        target_node = str(self._widget.comboBoxDestination.currentText())
        self.pub.publish(SourceTargetNodes(start_node, target_node))
        

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
