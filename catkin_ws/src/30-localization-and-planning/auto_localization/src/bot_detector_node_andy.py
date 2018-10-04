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

		self.MAXAREA = 50000
		self.MINAREA = 3000
		self.cam_info = rospy.wait_for_message("/mocap02/camera_node/raw_camera_info", CameraInfo, timeout=None)


		# initailize opencv background subtraction tool
		self.background_subtraction = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=50, detectShadows=False)

		#Publisher
		self.pub_result = rospy.Publisher("~image_mask", Image, queue_size=1)
		self.pub_cam_info = rospy.Publisher("~camera_info",CameraInfo,queue_size=1)

		#Subscriber
		self.sub_image = rospy.Subscriber("/mocap02/camera_node/image/rect", Image, self.cbImage, queue_size=10)
		

	# def cbCamInfo(self,caminfo_msg):
	# 	if not self.active:
	# 		return

	# 	self.cam_info = caminfo_msg

	def cbImage(self, image_msg):

		cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'mono8')

		img_mask = self.background_subtraction.apply(cv_image)
		#img_with_mask = cv2.bitwise_and(cv_image, img_mask)

		kernel = np.ones((3,3),np.uint8)
		img_mask = cv2.dilate(img_mask,kernel,iterations = 1)
		img_mask = cv2.morphologyEx(img_mask, cv2.MORPH_CLOSE, kernel)

		thresh = cv2.threshold(img_mask, 60, 255, cv2.THRESH_BINARY)[1]
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_CCOMP  ,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[1]

		img_process = np.full_like(cv_image,255)
		for c in cnts:		
			area = cv2.contourArea(c) 					
			if self.MAXAREA >= area >= self.MINAREA:	
				rect = cv2.minAreaRect(c)
				box = cv2.boxPoints(rect) 
				box = np.int0(box)
				#cv2.drawContours(cv_image,[box], 0, (255, 0, 0), 2)
				Xs = [i[0] for i in box]
				Ys = [i[1] for i in box]
				x1 = min(Xs)
				x2 = max(Xs)
				y1 = min(Ys)
				y2 = max(Ys)
				for i in range(640):
				 	for j in range(480):
				 		if  x1 <= i <= x2 and y1 <= j <= y2:
				 			img_process[j,i] = cv_image[j,i]

		img_pub = self.bridge.cv2_to_imgmsg(img_process, 'mono8')



		img_pub.header.stamp = image_msg.header.stamp
		img_pub.header.frame_id = image_msg.header.frame_id
		self.cam_info.header = image_msg.header
		self.pub_result.publish(img_pub)
		self.pub_cam_info.publish(self.cam_info)


	def on_Shutdown(self):
		self.loginfo("Shutdown.")


if __name__ == '__main__':
	rospy.init_node('bot_detector', anonymous=False)
	bot_detector_node=BotDetectorNode()
	rospy.on_shutdown(bot_detector_node.on_Shutdown)
	rospy.spin()
