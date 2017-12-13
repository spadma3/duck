#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_utils import get_base_name, rgb_from_ros
from object_detection.msg import ObjectDectectionList
from object_detection import ObjectDetector


class ObjectDetectionNode(object):
    def __init__(self):
        self.node_name = "Object Detection"
        self.active = True
        self.object_detector = ObjectDetector()
        self.t_last_update = rospy.get_time()

        # Subscribers
        self.sub_image = rospy.Subscriber("~camera_node/image/compressed", CompressedImage, self.on_image_received, queue_size=1)

        # Publishers

        self.pub_detection = rospy.Publisher("~detection_node/detections", ObjectDectectionList)

        # timer for updating the params
        # self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

    def on_image_received(self, compressed_image):
        detections = self.object_detector.detect(rgb_from_ros(compressed_image))
        # publish detections

        # self.pub_detection.publish()

    def onShutdown(self):
        self.object_detector.finalize()
        rospy.loginfo("[ObjectDetectionNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter',anonymous=False)
    object_detection_node = ObjectDetectionNode()
    rospy.on_shutdown(object_detection_node.onShutdown)
    rospy.spin()
