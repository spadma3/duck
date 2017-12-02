#!/usr/bin/env python

class MappingTile:
    tile_length = 200  # TODO: get this number
    _tile = None

    def __init__(self, tile):
        self._tile = tile

    @property
    def tile_center_location(self):
        return [self._tile.x * self.tile_length + self.tile_length / 2.0, self._tile.y * self.tile_length + self.tile_length / 2.0]

    def distance_to(self, position_2d):
        return np.sqrt(np.sum(np.square(self.tile_center_location - position_2d)))

    def node_from_duckiebot_rotation(self, db_rotation):
        diff_rotation = (db_rotation - self._tile.rotation) % 360

        # following the convention of how nodes are numbered at intersections, this holds. Yes, not very good design.
        if diff_rotation == 0:
            return self._tile.node5.name
        elif diff_rotation == 90:
            return self._tile.node3.name
        elif diff_rotation == 180:
            return self._tile.node1.name
        elif diff_rotation == 270:
            return self._tile.node7.name


class IntersectionMapper:
    mapping_tiles = []
    max_radius = 1  # in tile lenghts. If the 2d position is within this radius, it will be assign to this intersection. Must be <= 1.

    def __init__(self, graph_creator):
        for tile in graph_creator.tile_map:
            if 'way' in tile.type:
                self.mapping_tiles.append(MappingTile(tile))

    def get_node_name(self, position_2d, orientation):
        """
        given a position and a rotation of a duckiebot that is at a stop line (not checked),
        it returns the corresponding node on the graph.
        :param position_2d: [x,y]
        :param orientation: rotation on map, a float
        :return: node number as int
        """
        for intersection in self.mapping_tiles:
            if intersection.distance_to(position_2d) < self.max_radius * intersection.tile_length:
                return intersection.node_from_duckiebot_rotation(orientation)

        return None


if __name__ == '__main__':
    # for testing
    import os
    import numpy as np
    from generate_duckietown_map import graph_creator
    import tf
    import rospy
    print('AHAAAA')
    rospy.init_node('location_to_graph_mapping')
    listener = tf.TransformListener()

    # load csv
    script_dir = os.path.dirname(__file__)
    csv_filename = 'tiles_lab'
    map_path = os.path.abspath(script_dir + '/../../src/')

    gc = graph_creator()
    gc.build_graph_from_csv(map_path, csv_filename)
    rospy.loginfo('started!!')
    mapper = IntersectionMapper(gc)
    while True:
        rospy.sleep(5.0)
        #listener.waitForTransform('duckiebot', 'world', rospy.Time(), rospy.Duration(4.0))
        try:
            (trans, rot) = listener.lookupTransform('duckiebot', 'world', rospy.Time(0))
        except:
            pass
        node = mapper.get_node_name(np.array([500, 800]).reshape(-1, 1), 180)
        rospy.loginfo(node)

    rospy.on_shutdown(actions_dispatcher_node.onShutdown)
    rospy.spin()
