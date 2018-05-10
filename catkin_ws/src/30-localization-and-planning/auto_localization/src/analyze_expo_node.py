#!/usr/bin/env python
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from rosgraph_msgs.msg import Clock

import cv2
import rospy
import numpy as np

TEST_TIME = 115

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
		self.sub_clock = rospy.Subscriber("/clock", Clock, self.getTime, queue_size=1)

		#gate for closing call back function
		self.finish = False

		#record starting time
		self.start_time = -1
	
	def getBots(self, msg):

		if self.finish:
			return

		seq = msg.header.seq - 1 #robot seq start with 1
		print 'bot seq = ', seq

		if msg.data:
			el = 1
		else:
			el = 0

		self.bot_detection = self.datalist(self.bot_detection, el, seq)

	def getTags(self, msg):

		if self.finish:
			return

		el = 0
		
		seq = msg.header.seq  #tag seq start with 1
		print 'tag seq = ', seq
		for i in range(len(msg.detections)):
			if msg.detections[i].id[i] == 0:
				print 'get AT'
				el = 1
			else:
				el = 0

		self.tag_detection = self.datalist(self.tag_detection, el, seq)

	def getTime(self, msg):
		
		if self.finish:
			return

		if self.start_time == -1:
			self.start_time = msg.clock.secs

		secs = msg.clock.secs - self.start_time
		#self.loginfo('It has been %d secs.' % (secs))

		if secs > TEST_TIME:
			self.finish = True
			self.computeResult()

		
	def datalist(self, li, el, index):

		while len(li) < (index+1):
			if len(li) == index:
				li.append(el)
			else:
				li.append(-1)

		return li

	def computeResult(self):
		
		bot_detection = self.bot_detection
		tag_detection = self.tag_detection

		print bot_detection
		print tag_detection

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

		ratio = float(100.*float(count_tag_bot) / float(count_bot))
		print 'the ratio is ', ratio

		self.loginfo('The comparison is done, you can shutdown the node.')

	def loginfo(self, s):
		rospy.loginfo('[%s] %s' % (self.node_name, s))

	def on_Shutdown(self):
		self.loginfo("Shutdown.")


if __name__ == '__main__':
	rospy.init_node('analyze_expo', anonymous=False)
	analyze_expo_node=AnalyzeExpo()
	rospy.on_shutdown(analyze_expo_node.on_Shutdown)
	rospy.spin()