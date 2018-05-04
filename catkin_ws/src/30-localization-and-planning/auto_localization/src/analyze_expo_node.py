#!/usr/bin/env python
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection

import cv2
import rospy
import numpy as np

class AnalyzeExpo(object):
	"""docstring for AnalyzeExpo"""
	def __init__(self):
		self.node_name = "AnalyzeExpoNode"

		#Data gets from bot_detection and tag_detection
		self.bot_detection = []
		self.tag_detection = []

		#Subscriber and Publisher
		self.sub_bot_detect = rospy.Subscriber("bot_detection", BoolStamped, self.getBots, queue_size=1)
		self.sub_tag_detect = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.getTags, queue_size=1)
	
	def getBots(self, msg):

		if msg.data:
			el = 1
		else:
			el = 0

		self.bot_detection = self.datalist(self.bot_detection, el, msg.header.seq)

	def getTags(self, msg):
		
		if msg.detections.id == 0:
			el = 1
		else:
			el = 0
			
		self.tag_detection = self.datalist(self.tag_detection, el, msg.header.seq)
		
	def datalist(self, li, el, index):

		while len(li) < (index+1):
			if len(li) == index:
				li.append(el)
			else:
				li.append(-1)

	def computeResult(self):
		
		bot_detection = self.bot_detection
		tag_detection = self.tag_detection

		count_bot = 0
		count_tag_bot = 0

		for i in range(len(bot_detection)):
			if i >= len(tag_detection):
				if bot_detection[i] == 1:
					count_bot+=1
			else:
				if bot_detection[i] == tag_detection[i] == 1:
					count_tag_bot+=1
				if  bot_detection[i] == 1:
					count_bot+=1

		ratio = (float)count_tag_bot / (float)count_bot

		self.loginfo('The ratio of detection is %d' % (ratio))

	def loginfo(self, s):
		rospy.loginfo('[%s] %s' % (self.node_name, s))

	def on_Shutdown(self):
		self.loginfo("Shutdown.")


if __name__ == '__main__':
	rospy.init_node('analyze_expo', anonymous=False)
	analyze_expo_node=AnalyzeExpo()
	rospy.on_shutdown(analyze_expo_node.on_Shutdown)
	rospy.spin()