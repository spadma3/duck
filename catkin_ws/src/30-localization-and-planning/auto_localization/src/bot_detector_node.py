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
########################################################WTF#############
#firstframe = None
########################################################################
class BotDetectorNode(object):
	"""docstring for BotDetectorNode"""
	def __init__(self):
		self.node_name = "BotDetectorNode"

		#Constructor of bot detector
		self.bridge = CvBridge()
		self.firstframe = None
		self.MAXAREA = 50000
		self.MINAREA = 3000
		self.cam_info = rospy.wait_for_message("camera_info", CameraInfo, timeout=None)

		self.active = rospy.get_param('~bot_detection', 'false')

		# initailize opencv background subtraction tool
		self.background_subtraction = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

		#Publisher
		self.pub_result = rospy.Publisher("~image/rect", Image, queue_size=1)
		self.pub_cam_info = rospy.Publisher("~rect_camera_info",CameraInfo,queue_size=1)

		#Subscriber
		self.sub_image = rospy.Subscriber("image_rect", Image, self.cbImage, queue_size=10)


	# def cbCamInfo(self,caminfo_msg):
	# 	if not self.active:
	# 		return

	# 	self.cam_info = caminfo_msg

	def cbImage(self, image_msg):

		if not self.active:
			self.cam_info.header = image_msg.header
			self.pub_result.publish(image_msg)
			self.pub_cam_info.publish(self.cam_info)
			return

		cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')

		tStart = time.time()

		img_mask = self.background_subtraction.apply(cv_image)
		#img_with_mask = cv2.bitwise_and(cv_image, img_mask)
		tEndback = time.time()


		kernel = np.ones((3,3),np.uint8)
		img_mask = cv2.dilate(img_mask,kernel,iterations = 1)
		img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)

		'''
		gray=cv2.GaussianBlur(cv_image,(21,21),0)
		if self.firstframe is None:
		    self.firstframe = gray

		frameDelta = cv2.absdiff(self.firstframe,gray)
		thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
		thresh = cv2.dilate(thresh, None, iterations=2)

		'''
		x,y,w,h=cv2.boundingRect(img_mask)

		img_process = np.full_like(cv_image,255)

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
