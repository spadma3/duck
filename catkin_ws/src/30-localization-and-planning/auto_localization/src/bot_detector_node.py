#!/usr/bin/env python
import time
import cv2

from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from duckietown_msgs.msg import BoolStamped
import rospkg


class BotDetectorNode(object):
	"""docstring for BotDetectorNode"""
	def __init__(self):
		self.node_name = "BotDetectorNode"

		#Constructor of bot detector
		self.bridge = CvBridge()

		self.cam_info = rospy.wait_for_message("camera_info", CameraInfo, timeout=None)

		self.active = rospy.get_param('~bot_detection', 'false')

		self.last_img = None

		# initailize opencv background subtraction tool
		self.background_subtraction = cv2.createBackgroundSubtractorMOG2(history=450, varThreshold=70, detectShadows=False)

		#Publisher
		self.pub_result = rospy.Publisher("~image/rect", Image, queue_size=1)
		self.pub_cam_info = rospy.Publisher("~rect_camera_info",CameraInfo,queue_size=1)

		#Subscriber
		self.sub_image = rospy.Subscriber("image_rect", Image, self.cbImage, queue_size=10)


	def cbImage(self, image_msg):

		if not self.active:
			self.cam_info.header = image_msg.header
			self.pub_result.publish(image_msg)
			self.pub_cam_info.publish(self.cam_info)
			return

		cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')

		if self.last_img == None:
			self.last_img = np.full_like(cv_image, 0)

		tStart = time.time()

		img_mask = self.background_subtraction.apply(cv_image)
		#img_with_mask = cv2.bitwise_and(cv_image, img_mask)

		tEndback = time.time()
		img_process = np.full_like(cv_image,0)
		if(cv2.countNonZero(img_mask)>30):
			kernel = np.ones((3,3),np.uint8)
			img_mask = cv2.dilate(img_mask,kernel,iterations = 1)
			img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)

			x,y,w,h=cv2.boundingRect(img_mask)
			img_process[y:y+h,x:x+w] = cv_image[y:y+h,x:x+w]


		tEndtotal = time.time()

		print "Background subtraction = ", tEndback - tStart
		print "Total = ", tEndtotal - tStart
		img_pub = self.bridge.cv2_to_imgmsg(img_process, 'mono8')

		img_pub.header.stamp = image_msg.header.stamp
		img_pub.header.frame_id = image_msg.header.frame_id
		print self.cam_info
		self.cam_info.header = image_msg.header
		print self.cam_info
		self.pub_result.publish(img_pub)
		self.pub_cam_info.publish(self.cam_info)


	def on_Shutdown(self):
		self.loginfo("Shutdown.")


if __name__ == '__main__':
	rospy.init_node('bot_detector', anonymous=False)
	bot_detector_node=BotDetectorNode()
	rospy.on_shutdown(bot_detector_node.on_Shutdown)
	rospy.spin()
