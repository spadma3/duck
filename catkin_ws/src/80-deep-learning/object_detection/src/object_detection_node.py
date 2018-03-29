#!/usr/bin/env python
from os import path
import rospy
import rospkg
from sensor_msgs.msg import CompressedImage
from duckietown_utils import rgb_from_ros
from object_detection.msg import ObjectDetectionList, Detection
from object_detection import ObjectDetector


class ObjectDetectionNode:
    def __init__(self):
        self.node_name = "Object Detection"
        self.active = True

        # Detector Configuration
        rospack = rospkg.RosPack()
        self.configuration = rospy.get_param('~object_detector')  # TODO: On-the-fly reconfiguration
        # TODO: Fix after a place for inference models is established
        self.object_detector = ObjectDetector(
            path.join(rospack.get_path('object_detection'), self.configuration['inference_graph_path']),
            float(self.configuration['score_threshold']),
            bool(self.configuration['denormalize_boundingbox'])
        )

        # Subscribers
        self.sub_image = rospy.Subscriber("~image_compressed", CompressedImage, self.on_image_received, queue_size=1)

        # Publishers
        self.pub_detection = rospy.Publisher("~detections", ObjectDetectionList, queue_size=1)

        # timer for updating the params
        # self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

    def on_image_received(self, compressed_image):
        detections = self.object_detector.detect(rgb_from_ros(compressed_image))

        if len(detections) > 0:
            detection_list_msg = ObjectDetectionList()
            for detection in detections:
                detection_list_msg.detections.append(
                    Detection(class_label=detection['class_label'],
                              class_id=detection['class_id'],
                              xmin=detection['xmin'],
                              xmax=detection['xmax'],
                              ymin=detection['ymin'],
                              ymax=detection['ymax'],
                              score=detection['score'])
                )

            self.pub_detection.publish(detection_list_msg)

    def onShutdown(self):
        self.object_detector.finalize()
        rospy.loginfo("[ObjectDetectionNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=False)
    object_detection_node = ObjectDetectionNode()
    rospy.on_shutdown(object_detection_node.onShutdown)
    rospy.spin()
