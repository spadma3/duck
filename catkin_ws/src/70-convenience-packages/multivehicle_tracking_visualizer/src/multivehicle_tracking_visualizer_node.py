#!/usr/bin/env python
from random import uniform, randint

import threading

import math
import rospy
import tf
from multivehicle_tracker.msg import Tracklet, TrackletList
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from duckietown_msgs.msg import LanePose


class MultiVehicleTrackerVisualizerNode:
    def __init__(self):
        self.node_name = "Multi Vehicle Tracking Visualizer"

        self.tracked_objects = {}
        self.lock = threading.Lock()
        self.veh_name = rospy.get_param('~veh_name')
        self.rand_gen = uniform
        # Subscribers
        self.sub_detections = rospy.Subscriber("~tracking", TrackletList, self.on_tracklets_received, queue_size=1)
        self.pose_detection = rospy.Subscriber("~pose", LanePose, self.on_pose, queue_size=1)
        # Publishers
        self.pub_tracklet = rospy.Publisher("~tracking_visualizations", MarkerArray, queue_size=1)

    def create_self_marker(self, lane_pose_msg):
        marker = self.create_colored_marker_msg((1.0, 1.0, 1.0), None, identifier=429496729)
        yaw_quat = tf.transformations.quaternion_about_axis(-lane_pose_msg.phi, [0, 0, 1])
        marker.pose.orientation.x = yaw_quat[0]
        marker.pose.orientation.y = yaw_quat[1]
        marker.pose.orientation.z = yaw_quat[2]
        marker.pose.orientation.w = yaw_quat[3]

        marker.pose.position.x = 0.0
        marker.pose.position.y = -lane_pose_msg.d
        marker.pose.position.z = 0.0

        return marker

    def create_name_marker(self):
        marker = Marker()

        marker.header.frame_id = self.veh_name
        marker.id = 10000000
        marker.action = Marker.ADD
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = self.veh_name
        marker.color = ColorRGBA(r=0, g=0.7, b=0.7, a=0.8)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.1
        # marker.lifetime = rospy.Duration.from_sec(0.2)

        return marker


    def on_pose(self, lane_pose_msg):
        reference_msg = MarkerArray()
        self_marker = self.create_self_marker(lane_pose_msg)
        text_marker = self.create_name_marker()

        reference_msg.markers.append(self_marker)
        reference_msg.markers.append(text_marker)

        self.pub_tracklet.publish(reference_msg)

    def create_colored_marker_msg(self, color, tracklet, identifier=None):
        marker = Marker()

        marker.header.stamp = rospy.Time()
        marker.header.frame_id = self.veh_name
        if identifier is None:  # drawing a particle
            marker.id = randint(4098, 1000000)
            marker.lifetime = rospy.Duration.from_sec(1.0)
            marker.type = Marker.SPHERE
            marker.scale.x = math.sqrt(tracklet.sigma_x)
            marker.scale.y = math.sqrt(tracklet.sigma_y)
            marker.scale.z = 0.05
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
        else:
            marker.id = identifier
            marker.type = Marker.MESH_RESOURCE
            marker.lifetime = rospy.Duration.from_sec(0.5)
            marker.mesh_resource = "package://multivehicle_tracking_visualizer/meshes/duckiebot.dae"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

        marker.ns = self.veh_name + '/tracking'
        marker.action = Marker.ADD

        if color:
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)

        if tracklet is not None:
            marker.pose.position.x = tracklet.x
            marker.pose.position.y = tracklet.y
            marker.pose.position.z = 0.0
            heading = math.pi / 2 - tracklet.heading  # transform to local object's coordinates frame
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(heading / 2)  # rotation on z
            marker.pose.orientation.w = math.cos(heading / 2)  # quaternion w
        else:
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0  # quaternion w

        return marker

    def delete_marker(self, marker_id):
        marker = Marker()

        marker.header.stamp = rospy.Time()
        marker.header.frame_id = self.veh_name
        marker.ns = self.veh_name + '/tracking'
        marker.id = marker_id
        marker.action = Marker.DELETE
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def on_tracklets_received(self,  tracklet_list):
        markers_msg = MarkerArray()
        with self.lock:
            to_remove = []
            for tracklet in tracklet_list.tracklets:
                if tracklet.id not in self.tracked_objects:
                    self.tracked_objects[tracklet.id] = (self.rand_gen(0, 1), self.rand_gen(0, 1), self.rand_gen(0, 1))
                if tracklet.status == Tracklet.STATUS_TRACKING or tracklet.status == Tracklet.STATUS_BORN:
                    color = self.tracked_objects[tracklet.id]
                    markers_msg.markers.append(self.create_colored_marker_msg(color, tracklet, hash(tracklet.id) % 4097))  # 2^32 - 1
                    markers_msg.markers.append(self.create_colored_marker_msg(color, tracklet))
                else:
                    markers_msg.markers.append(self.delete_marker(hash(tracklet.id) % 4097))
                    to_remove.append(tracklet.id)
            self.pub_tracklet.publish(markers_msg)

            for trackled_id in to_remove:
                self.tracked_objects.pop(trackled_id)

    def onShutdown(self):
        rospy.loginfo("[MultiVehicleTrackingVisualizer] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('multivehicle_tracking_visualizer', anonymous=False)
    multi_vehicle_tracker_visualizer_node = MultiVehicleTrackerVisualizerNode()
    rospy.on_shutdown(multi_vehicle_tracker_visualizer_node.onShutdown)
    rospy.spin()
