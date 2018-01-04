#!/usr/bin/env python

import numpy as np
import rospy
from object_detection.msg import ObjectDetectionList
from multivehicle_tracker.msg import Tracklet, TrackletList
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
        self.pub_tracklet = rospy.Publisher("~tracking", TrackletList, queue_size=1)

        # timer for updating the params
        self.continuous_prediction = rospy.Timer(rospy.Duration.from_sec(0.1), self.on_delta_t)
        self.garbage_collection = rospy.Timer(rospy.Duration.from_sec(0.2), self.garbage_collection)

    def garbage_collection(self, event):
        lost_tracklets = self.multivehicle_tracker.get_tracklets_info(Tracklet.STATUS_LOST)
        self.publish_tracklets(lost_tracklets)
        self.multivehicle_tracker.clean_dead()

    def on_delta_t(self, event):
        self.multivehicle_tracker.time_elapsed(timestamp=rospy.get_time())

    def pixel2ground(self, x, y):
        u, v, w = np.dot(self.groundplane_homography, (x, y, 1))
        return u / w, v / w  # TODO: switch coordinates

    def mid_groundplane_point(self, bot_detection):
        y_max = bot_detection.ymax

        left = self.pixel2ground(bot_detection.xmin, y_max)
        right = self.pixel2ground(bot_detection.xmax, y_max)

        return (left[0] + right[0]) / 2, (left[1] + right[1]) / 2

    def publish_tracklets(self, tracklets_info):
        tracklet_list_msg = TrackletList()
        for tracklet_info in tracklets_info:
            tracklet_list_msg.tracklets.append(
                Tracklet(
                    id=str(tracklet_info['id']),
                    status=tracklet_info['status'],  # TODO: grab it
                    x=tracklet_info['x'],
                    y=tracklet_info['y'],
                    heading=tracklet_info['phi'],
                    velocity=tracklet_info['v'],
                    sigma_x=tracklet_info['sigma_x'],
                    sigma_y=tracklet_info['sigma_y']
                )
            )

        if len(tracklet_list_msg.tracklets) > 0:
            self.pub_tracklet.publish(tracklet_list_msg)

    def on_detections_received(self, object_detections):
        current_time = rospy.get_time()

        bot_detections = filter(self.bot_detection_filter, object_detections.detections)

        for bot_detection in bot_detections:
            position = self.mid_groundplane_point(bot_detection)
            self.multivehicle_tracker.add_detection(position[0], position[1], bot_detection.score, current_time)

        self.publish_tracklets(self.multivehicle_tracker.get_tracklets_info())

    def onShutdown(self):
        rospy.loginfo("[MultiVehicleTracker] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('multivehicle_tracker', anonymous=False)
    multi_vehicle_tracker_node = MultiVehicleTrackerNode()
    rospy.on_shutdown(multi_vehicle_tracker_node.onShutdown)
    rospy.spin()
