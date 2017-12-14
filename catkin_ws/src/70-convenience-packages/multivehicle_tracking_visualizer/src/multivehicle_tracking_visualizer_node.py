#!/usr/bin/env python
from random import uniform

import rospy
from multivehicle_tracker.msg import TrackletList
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class MultiVehicleTrackerVisualizerNode:
    def __init__(self):
        self.node_name = "Multi Vehicle Tracking Visualizer"

        self.tracked_objects = {}
        self.veh_name = rospy.get_param('veh_name', 'patito')
        self.rand_gen = uniform
        # Subscribers
        self.sub_detections = rospy.Subscriber("~tracking", TrackletList, self.on_tracklets_received, queue_size=1)

        # Publishers
        self.pub_tracklet = rospy.Publisher("~tracking_visualizations", MarkerArray, queue_size=1)

    def create_colored_marker_msg(self, color, tracklet):
        marker = Marker()

        marker.header.stamp = rospy.Time()
        marker.header.frame_id = self.veh_name
        marker.ns = self.veh_name + '/tracking'
        marker.lifetime = rospy.Duration.from_sec(3.0)
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = tracklet.x
        marker.pose.position.y = tracklet.y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        return marker

    def on_tracklets_received(self,  tracklet_list):
        markers_msg = MarkerArray()
        for tracklet in tracklet_list.tracklets:
            if tracklet.id not in self.tracked_objects:
                self.tracked_objects[tracklet.id] = (self.rand_gen(0, 1), self.rand_gen(0, 1), self.rand_gen(0, 1))
            color = self.tracked_objects[tracklet.id]
            markers_msg.markers.append(self.create_colored_marker_msg(color, tracklet))

        self.pub_tracklet.publish(markers_msg)

    def onShutdown(self):
        rospy.loginfo("[MultiVehicleTrackingVisualizer] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('multivehicle_tracking_visualizer', anonymous=False)
    multi_vehicle_tracker_visualizer_node = MultiVehicleTrackerVisualizerNode()
    rospy.on_shutdown(multi_vehicle_tracker_visualizer_node.onShutdown)
    rospy.spin()
