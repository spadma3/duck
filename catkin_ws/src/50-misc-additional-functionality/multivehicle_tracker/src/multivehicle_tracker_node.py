#!/usr/bin/env python

import numpy as np
import rospy
from object_detection.msg import ObjectDetectionList
from multivehicle_tracker.msg import Tracklet
from multivehicle_tracker import NaiveMultitargetTracker
from duckietown_utils import load_homography


class MultiVehicleTrackerNode:
    def __init__(self):
        self.node_name = "Multi Vehicle Tracker"
        self.active = True

        # Tracker Configuration
        self.configuration = rospy.get_param('~multivehicle_tracker')  # TODO: On-the-fly reconfiguration
        self.bot_detection_filter = lambda detection: detection.class_id == self.configuration['detection_info']['id']

        self.multivehicle_tracker = NaiveMultitargetTracker()

        self.groundplane_homography = load_homography(rospy.get_param('~vehicle_name'))

        # Subscribers
        self.sub_detections = rospy.Subscriber("~object_detections", ObjectDetectionList, self.on_detections_received, queue_size=1)

        # Publishers
        self.pub_tracklet = rospy.Publisher("~tracking", Tracklet, queue_size=1)

        # timer for updating the params
        self.continuous_prediction = rospy.Timer(rospy.Duration.from_sec(0.1), self.on_delta_t)

    def on_delta_t(self, event):
        self.multivehicle_tracker.time_elapsed(timestamp=event.current_real)

    def pixel2ground(self, x, y):
        u, v, w = np.dot(self.groundplane_homography, (x, y, 1))
        return u / w, v / w

    def closest_groundplane_point(self, bot_detection):
        y_max = bot_detection.ymax

        left = self.pixel2ground(bot_detection.xmin, y_max)
        right = self.pixel2ground(bot_detection.xmax, y_max)

        if np.linalg.norm(left) < np.linalg.norm(right):
            return left

        return right

    def on_detections_received(self, object_detections):
        current_time = rospy.get_time()

        bot_detections = filter(self.bot_detection_filter, object_detections.detections)

        for bot_detection in bot_detections:
            position = self.closest_groundplane_point(bot_detection)
            self.multivehicle_tracker.add_detection(position[0], position[1], bot_detection.score, current_time)

        for tracklet_info in self.multivehicle_tracker.get_tracklets_info():
            tracklet_msg = Tracklet()
            tracklet_msg.x = tracklet_info['x']
            tracklet_msg.y = tracklet_info['y']
            tracklet_msg.heading = tracklet_info['phi']
            tracklet_msg.velocity = tracklet_info['v']
            tracklet_msg.status = Tracklet.STATUS_TRACKING  # TODO: grab it
            self.pub_tracklet.publish(tracklet_msg)

    def onShutdown(self):
        rospy.loginfo("[MultiVehicleTracker] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('multivehicle_tracker', anonymous=False)
    multi_vehicle_tracker_node = MultiVehicleTrackerNode()
    rospy.on_shutdown(multi_vehicle_tracker_node.onShutdown)
    rospy.spin()
