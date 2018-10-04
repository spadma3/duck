#!/usr/bin/env python
import time
import cv2

from cv_bridge import CvBridge
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
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
		self.pub_cam_info = rospy.Publisher("~rect_camera_info",CameraInfo,queue_size=1)

		#Subscriber
		self.sub_image = rospy.Subscriber("/mocap01/camera_node/image/rect", Image, self.cbImage, queue_size=1)
		self.sub_cam_info = rospy.Subscriber("/mocap01/camera_node/raw_camera_info",CameraInfo,self.cbCamInfo,queue_size=1)

	def cbCamInfo(self,caminfo_msg):
		if not self.active:
			return

		self.cam_info = caminfo_msg

	def cbImage(self, image_msg):

		cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')

		img_mask = self.background_subtraction.apply(cv_image)
		#img_with_mask = cv2.bitwise_and(cv_image, img_mask)
		
		#contour!!!
		ret,thresh = cv2.threshold(img_mask,127,255,0)
		im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		#calculate center of gravity
		cnt=contours[0]
		M=cv2.moments(cnt)
		cx=int(M['m10']/M['m00'])
		cy=int(M['m01']/M['m00'])
		
		width=50
		img_mask = cv_image[cx-width:cx+width, cy-width:cy+width]


		img_pub = self.bridge.cv2_to_imgmsg(img_mask, 'mono8')
		img_pub.header.stamp = image_msg.header.stamp
		img_pub.header.frame_id = image_msg.header.frame_id

		self.pub_result.publish(img_pub)
		self.pub_cam_info.publish(self.cam_info)

	def on_Shutdown(self):
		self.loginfo("Shutdown.")


if _name_ == '_main_':
	rospy.init_node('bot_detector', anonymous=False)
	bot_detector_node=BotDetectorNode()
	rospy.on_shutdown(bot_detector_node.on_Shutdown)
	rospy.spin()