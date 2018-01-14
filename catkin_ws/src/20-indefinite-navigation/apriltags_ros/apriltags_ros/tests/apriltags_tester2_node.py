#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
from tf import transformations as tr
import numpy as np

class ApriltagsTesterNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('apriltags_tester_node', anonymous=False)
        self.msg_received = False
        self.msg_tags = AprilTagDetectionArray()

        # Setup the publisher and subscriber
        self.pub_raw  = rospy.Publisher("~image_rect", Image, queue_size=1, latch=True)
        self.pub_info  = rospy.Publisher("~camera_info", CameraInfo, queue_size=1, latch=True)
        self.sub_tag = rospy.Subscriber( "~apriltags", AprilTagDetectionArray, self.tagCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.pub_raw.get_num_connections() < 1 or self.pub_info.get_num_connections() < 1 or
                self.sub_tag.get_num_connections() < 1) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tagCallback(self, msg_tags):
        self.msg_tags = msg_tags
        self.msg_received = True

    def test_publisher_and_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_raw.get_num_connections(), 1, "No connections found on image_raw topic")
        self.assertGreaterEqual(self.pub_info.get_num_connections(), 1, "No connections found on camera_info topic")
        self.assertGreaterEqual(self.sub_tag.get_num_connections(), 1, "No connections found on apriltags topic")

    def test_with_known_image20(self):
        filename = rospy.get_param("~filename")
        self.setup()    # Setup the node

        # Publish the camera info
        msg_info = CameraInfo()
        msg_info.height = 480
        msg_info.width = 640
        msg_info.K = [337.27528732814227, 0.0, 331.1903596380628, 0.0, 334.41950735662476, 246.80442900695877, 0.0, 0.0, 1.0]
        self.pub_info.publish(msg_info)

        # Publish the test image
        #img = cv2.imread(filename)
        img = cv2.imread("/home/nilsiism/duckietown/catkin_ws/src/20-indefinite-navigation/apriltags_ros/apriltags_ros/tests/apriltag_test_img_rect_dist_30.png")
        cvb = CvBridge()
        msg_raw = cvb.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_raw.publish(msg_raw)

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")

        # Check that an apriltag with id=108 was detected
        found = 0
        for tag in self.msg_tags.detections:

            print "Dectected tag ID: ", tag.id

            if tag.id == 128:
                found +=1
                self.assertAlmostEqual(tag.pose.pose.position.x, 0.0, delta=0.02) # Allow 5 cm of error margin
                self.assertAlmostEqual(tag.pose.pose.position.y, 0.0, delta=0.02) # Allow 5 cm of error margin
                self.assertAlmostEqual(tag.pose.pose.position.z, 0.35, delta=0.001) # Allow 5 cm of error margin
                # Convert the quat to an angle about z
                rot = tag.pose.pose.orientation
                ang = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[2] #z axis angle only
                self.assertAlmostEqual(ang, 0, delta=15*np.pi/180) # Allow up to 15 degrees of error margin
                
        self.assertGreaterEqual(found,1, "Expected apriltag with id=128 not found.")



if __name__ == '__main__':
    rostest.rosrun('apriltags', 'apriltags_tester_node', ApriltagsTesterNode)
