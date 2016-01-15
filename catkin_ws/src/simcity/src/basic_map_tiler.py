#!/usr/bin/env python
import rospy
from simcity.util import TileProduction # For producing tile messages. 
from std_msgs.msg import String #Imports msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospkg
#from duckietown_msgs.msg import MapTile
from duckietown_msgs.msg import GraphInfo

class BasicMapTiler(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initializing." %(self.node_name))

        # == Publishers == #
        # RViz publishers
        self.pub_markers = rospy.Publisher("visualization_marker",Marker, queue_size=1)
        self.pub_marker_arrays = rospy.Publisher("visualization_marker_array", MarkerArray, \
                                                 queue_size=1)
        # PathPlanning graph info publisher
        self.pub_graph_info = rospy.Publisher("graphingstuff", GraphInfo, queue_size=1)
        
        # == Subscribers == #
        #self.sub_marker_arrays = rospy.Subscriber("visualization_marker_array", MarkerArray, \
        #                                          self.cbTopic)
        
        # == Setup Parameters == #
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)

        # === Load tiles and maps === #
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('simcity')
        self.mapfile = self.setupParameter("~map_file", self.pkg_path+"/maps/map.yaml") 
        self.tilesfile = self.setupParameter("~tiles_file", self.pkg_path+"/tiles/tiles.yaml")

        
        # Create a timer that calls the cbTimer function every 1.0 second
        self.tile_factory = TileProduction(self.mapfile, self.tilesfile)
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        self.tile_index = 0

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))

        return value

    def cbTopic(self,msg):
        rospy.loginfo("[%s] Received markarray number %s" %(self.node_name, self.tile_index))
        self.tile_index += 1

    # Does the publishing, every second. 
    # Sleeps until Rviz awakes
    def cbTimer(self,event):
        while (self.pub_marker_arrays.get_num_connections() < 1):
            if rospy.is_shutdown():
                return 0
            rospy.loginfo("Please create a subscriber to the marker")
            self.tile_index = 0 # reset markerarray count
            rospy.sleep(5)
        #continue through the list of tiles in the map
        if self.pub_marker_arrays.get_num_connections() >= 1:
            if self.tile_index >= self.tile_factory.get_num_tiles_in_map():
                return # finished sending all tiles
            fullmsg, graphinfo = self.tile_factory.get_certain_tile_message(self.tile_index)
            self.pub_marker_arrays.publish(fullmsg) # fullmsg: MarkerArray
            self.pub_graph_info.publish(graphinfo)
            self.tile_index += 1 

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('basic_map_tiler', anonymous=False)

    # Create the NodeName object
    node = BasicMapTiler()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
