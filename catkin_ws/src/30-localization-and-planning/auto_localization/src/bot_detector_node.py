#!/usr/bin/env python
import time
import cv2

from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
import rospkg

BG_SAMPLE = 150

class BotDetectorNode(object):
	"""docstring for BotDetectorNode"""
	def __init__(self):
		self.node_name = "BotDetectorNode"

		#Constructor of bot detector
		self.bridge = CvBridge()

		self.active  = True

		# initailize opencv background subtraction tool
		self.background_subtraction = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

		#Publisher
		self.pub_result = rospy.Publisher("~image_mask", Image, queue_size=1)

		#Subscriber
		self.sub_image = rospy.Subscriber("image", Image, self.cbImage, queue_size=1)
		self.subswitch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)


	def cbSwitch(self, switch_msg):
		self.active = switch_msg.data

	def cbImage(self, image_msg):

		cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')

		img_mask = self.background_subtraction.apply(cv_image)
		#img_with_mask = cv2.bitwise_and(cv_image, img_mask)

		kernel = np.ones((3,3),np.uint8)
		img_mask = cv2.dilate(img_mask,kernel,iterations = 1)
		img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)

		img_pub = self.bridge.cv2_to_imgmsg(img_mask, 'mono8')
		img_pub.header.stamp = image_msg.header.stamp
		img_pub.header.frame_id = image_msg.header.frame_id

		self.pub_result.publish(img_pub)

	def on_Shutdown(self):
		self.loginfo("Shutdown.")


if __name__ == '__main__':
	rospy.init_node('bot_detector', anonymous=False)
	bot_detector_node=BotDetectorNode()
	rospy.on_shutdown(bot_detector_node.on_Shutdown)
	rospy.spin()
