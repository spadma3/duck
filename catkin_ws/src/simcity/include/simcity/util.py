# Import stuff here
import random
import rospy
import os
import yaml
import numpy as np
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from duckietown_msgs.msg import MapTile
from geometry_msgs.msg import Point

# Implement modules
class TileProduction(object):
    def __init__(self, mapfile, tilesfile):
        ''' 
        Initialize dictionary of tiles, perhaps with tile messages. 
        Initialize map of tiles.
        Initialize index at which the production line is currently
         at (ie which next tile of map must be sent).
        '''
        dict_file = open(tilesfile, 'r')
        self.tile_dict = yaml.load(dict_file)
        dict_file.close()
        
        map_file = open(mapfile, 'r') 
        self.map_list = yaml.load(map_file)
        map_file.close()

        self.num_tiles_in_map = len(self.map_list)

        self.next_map_index = 0 # next index of tile to send

    def get_num_tiles_in_map(self):
        return self.num_tiles_in_map

    def get_certain_tile_message(self, array_index):
        '''
        Gets tile_index tile from the map, puts it into duckietown_msgs/MapFile.
        Arg array_index MUST be within the bounds of map_list 
         when get_certain_tile_message is called.
        '''
        next_tile_info = self.map_list[array_index]
        tile_name = next_tile_info['tile_type']
        tile_center_position = next_tile_info['tile_center']
        
        return self.get_tile_message(tile_name, tile_center_position, 
                                     array_index)

    def get_next_tile_message(self):
        '''
v        Gets next tile from the map.
        '''
        next_tile_info = self.map_list[self.next_map_index]
        if self.next_map_index >= len(next_tile_info):
            return MarkerArray() 
        tile_name = next_tile_info['tile_type']
        tile_center_position = next_tile_info['tile_center']
        
        self.next_map_index += 1
        array_index = 0; # causes only one markerarray to be drawn
        return self.get_tile_message(tile_name, tile_center_position, array_index)

    def get_tile_message(self, tile_name, tile_center_position, array_index):
        # according to args, construct and return the appropriate tile message

        tile_info = self.tile_dict[tile_name] #for now, includes east_west//north_south 
        #and number of lanes at the end. might switch to id # later?

        tile_length = 1.0 # TODO rmata pull info from yaml? some config file?
        tile_origin_position = (np.array(tile_center_position) 
                                - np.array([tile_length/2, tile_length/2])).tolist()
        # lower leftmost corner of tile is the origin of the tile (convention)
        
        # construct the MapTile message
        tilemsg = MapTile()
        
        # construct the traffic MarkerArray
        traffic = MarkerArray()
        arrow_ids = range(len(tile_info['connectivity']))
        i = 0
        for lane in tile_info['connectivity']:
            # calculate start and end of arrow, as per lane
            arrow_start = (np.array(tile_info['nodes_positions'][lane[0]])*tile_length 
                           + np.array(tile_origin_position)).tolist()
            arrow_end = (np.array(tile_info['nodes_positions'][lane[1]])*tile_length 
                         + np.array(tile_origin_position)).tolist()
            # add the appropriate marker
            arrow = Marker()
            arrow.type = 0 # arrow
            arrow.action = 0 # add
            arrow.header.frame_id  = "/map"
            arrow.id = arrow_ids[i]+8*array_index; i+=1

            # scales of arrow
            arrow.scale.x = 0.025
            arrow.scale.y = 0.1
            arrow.scale.z = 0.1
            # start, end?
            arrow.points.append(Point(arrow_start[0], arrow_start[1], 0))
            arrow.points.append(Point(arrow_end[0], arrow_end[1], 0))
            # color
            arrow.color.r = 1.0
            arrow.color.b = 1.0
            arrow.color.g = 0.0
            arrow.color.a = 1.0
            traffic.markers.append(arrow)
        tilemsg.traffic_arrows = traffic
        
        # construct the lane edges MarkerArray
        lane_edges = MarkerArray()
        i = 0
        outside_lines = Marker() # line list
        center_lines = Marker() # line list
        # pull out lane edges for each tile: white is on the right, yellow on left
        for lane in tile_info['road_edges']:
            white_line_start = (np.array(tile_info['nodes_positions'][lane[0]])*tile_length 
                           + np.array(tile_origin_position)).tolist()
            white_line_end = (np.array(tile_info['nodes_positions'][lane[1]])*tile_length 
                         + np.array(tile_origin_position)).tolist()
            outside_lines.color.r = 1.0
            outside_lines.color.g = 1.0
            outside_lines.color.b = 1.0
            outside_lines.color.a = 1.0
            outside_lines.points.append(Point(white_line_start[0], white_line_start[1]))
            outside_lines.points.append(Point(white_line_end[0], white_line_end[1]))
        
        for lane in tile_info['road_center']:
            yellow_line_start = (np.array(tile_info['nodes_positions'][lane[0]])*tile_length 
                           + np.array(tile_origin_position)).tolist()
            yellow_line_end = (np.array(tile_info['nodes_positions'][lane[1]])*tile_length 
                         + np.array(tile_origin_position)).tolist()
            center_lines.color.r = 1.0
            center_lines.color.g = 1.0
            center_lines.color.b = 0.0
            center_lines.color.a = 1.0
            center_lines.points.append(Point(yellow_line_start[0], yellow_line_start[1]))
            center_lines.points.append(Point(yellow_list_end[0], yellow_line_end[1]))

        lane_edges.markers.append(outside_lines)
        lane_edges.markers.append(center_lines)

        # construct stop lines MarkerArray...TODO rmata
        
        tilemsg.lane_edges = lane_edges
        return tilemsg

if __name__ == '__main__':
    # Test code for ModuleNames
    print 'No test code for simcity in main yet.'

