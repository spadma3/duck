#!/usr/bin/env python
import sys

class PixelAndMapTransformer():
    def __init__(self,tile_size,image_height):
        self.tile_size = float(tile_size) # to prevent python from rounding prematurely
        self.tiles_y = int(float(image_height)/self.tile_size)

    # this maps a zero-based tile location to a tile-midpoint in pixel coordinates
    def map_to_image(self, point_on_map):
        return (int((point_on_map[0])* self.tile_size),
                int((self.tiles_y - point_on_map[1]) * self.tile_size))

    # this maps a pixel point to the zero-based tile location the pixel is in
    def image_to_map(self, point_on_image):
        y_temp = self.tiles_y - float((point_on_image[1] + 1) / self.tile_size)
        x_temp = float(point_on_image[0] / self.tile_size)
        return (x_temp,y_temp)

class MapToGraphTransformer():
    def __init__(self, duckietown_graph):
        self.duckietown_graph = dict()
        for node_name, node_pos in duckietown_graph.node_positions.iteritems():
            if (node_name[0:4] == "turn" or  int(node_name) % 2 == 0):
                continue
            else:
                self.duckietown_graph.update({node_name: node_pos})

    def get_closest_node(self, map_position):
        min_distance = sys.float_info.max
        closest_node = -1
        for node_name, node_pos in self.duckietown_graph.iteritems():
            manhattan_dis = abs(node_pos[0] - map_position[0]) + abs(node_pos[1] - map_position[1])
            if (manhattan_dis < min_distance):
                min_distance = manhattan_dis
                closest_node = node_name
        return closest_node
