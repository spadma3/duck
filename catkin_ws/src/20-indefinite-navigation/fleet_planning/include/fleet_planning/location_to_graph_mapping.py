#!/usr/bin/env python
import os, sys
import numpy as np
from generate_duckietown_map import graph_creator
import tf
import rospy
import tf2_ros


class MappingTile:
    tile_length = 0.595  # TODO: retrieve this from parameter file in duckietown/00-infrastructure/config/baseline/duckeitown_description
    _tile = None

    def __init__(self, tile):
        self._tile = tile

    @property
    def tile_center_location(self):
        return np.array([self._tile.x * self.tile_length + self.tile_length / 2.0, self._tile.y * self.tile_length + self.tile_length / 2.0])

    def distance_to(self, position_2d):
        return np.sqrt(np.sum(np.square(self.tile_center_location - position_2d)))

    def node_from_duckiebot_rotation(self, db_rotation):
        db_rotation = np.around(db_rotation, -1)

        rest = db_rotation % 90
        if rest > 45:
            db_rotation += 90 - rest
        else:
            db_rotation -= rest

        diff_rotation = (db_rotation - self._tile.rotation) % 360

        # following the convention of how nodes are numbered at intersections, this holds. Yes, not very good design.
        try:
            if diff_rotation == 180:
                return self._tile.node1.name
            elif diff_rotation == 270:
                return self._tile.node3.name
            elif diff_rotation == 90:
                return self._tile.node7.name
            elif diff_rotation == 0:
                return self._tile.node5.name
        except AttributeError:
            rospy.logwarn('Robot was located at illegal part of the intersection, localization ignored.')


class IntersectionMapper:
    def __init__(self, graph_creator):
        self.mapping_tiles = []
        self.max_radius = 1  # in tile lenghts. If the 2d position is within this radius, it will be assign to this intersection. Must be <= 1.

        for tile in graph_creator.tile_map:
            if 'way' in tile.type:
                self.mapping_tiles.append(MappingTile(tile))

    def get_node_name(self, position_2d_m, rotation_deg):
        """
        given a position and a rotation of a duckiebot that is at a stop line (not checked),
        it returns the corresponding node on the graph.
        :param position_2d_m: [x,y] in meters
        :param rotation_deg: z rotation in degrees on map, a float
        :return: node number as int
        """
        for intersection in self.mapping_tiles:
            if intersection.distance_to(position_2d_m) < self.max_radius * intersection.tile_length:
                return intersection.node_from_duckiebot_rotation(rotation_deg)
        return None

if __name__ == '__main__':
    # for testing
    rospy.init_node('location_to_graph_mapping')
    listener = tf.TransformListener()

    # build graph from csv. This should be available in your class.
    script_dir = os.path.dirname(__file__)
    csv_filename = 'tiles_lab'
    map_path = os.path.abspath(script_dir + '/../../src/')

    gc = graph_creator()
    gc.build_graph_from_csv(map_path, csv_filename)

    rospy.loginfo('started!!')
    mapper = IntersectionMapper(gc)

    # localization mapping starts here. First get location message (=Transform). Then map to node name.
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('world', 'duckiebot', rospy.Time(), rospy.Duration(4.0))
        except tf2_ros.TransformException:
            rospy.logwarn('transform did not appear!')

        try:
            (trans, rot) = listener.lookupTransform('world', 'duckiebot', rospy.Time(0))
        except tf2_ros.LookupException:
            rospy.logwarn('transform could not be found')

        rot = tf.transformations.euler_from_quaternion(rot)[2]

        # do mapping from 2d position + rotation to node number.
        node = mapper.get_node_name(trans[:2], np.degrees(rot))
        print(node)
        rate.sleep()


