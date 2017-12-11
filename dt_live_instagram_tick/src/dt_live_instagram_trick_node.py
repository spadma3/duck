#!/usr/bin/env python

import rospy
import sys
import os

import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

from duckietown_utils import rgb_from_ros
from duckietown_utils import write_image_as_jpg
from duckietown_utils import d8_compressed_image_from_cv_image

class live_instagram(object):
	def __init__(self):i
		# Save the name of the node
		self.node_name = rospy.get_name()
		
		rospy.loginfo("[%s] Initialzing." %(self.node_name))
		
		# Read parameters
		self.topic = self.setupParameter('~image_topic', '/tesla/camera_node')
		self.filter = self.setupParameter('~filter', 'sepia')
		
		# Setup publisher
		self.pub_camera_images_filter = rospy.Publisher(self.topic + "filter/compressed", CompressedImage, queue_size=1)

		# Setup subscriber
		self.sub_camera_images = rospy.Subscriber(self.topic + 'compressed', CompressedImage, self.cbCameraImages)
		
		rospy.loginfo("[%s] Initialzed." %(self.node_name))

	def cbCameraImages(self, msg): 
		image = rgb_from_ros(msg)
		image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
		#for i in range(0, len(filters)):
		if(self.filter == 'flip-vertical'):
		    image = np.fliplr(image)
		elif(self.filter == 'flip-horizontal'):
		    image = np.flipud(image)
		elif(self.filter == 'grayscale'):
		    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		elif(self.filter == 'sepia'):
		    kernel = np.array([[0.272, 0.534, 0.131], [0.349, 0.686, 0.168], [0.393, 0.769, 0.189]])
		    image = cv2.transform(image, kernel)
		else:
		    print("Not valid filter!")
		    sys.exit()
		       
		new_msg = d8_compressed_image_from_cv_image(image)
		self.pub_camera_images_filter.publish(new_msg)		

	def on_shutdown(self):
		rospy.loginfo("[%s] Shutting down." %(self.node_name))

	def setupParameter(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

if __name__ == '__main__':
	# Initialize the node with rospy
	rospy.init_node('live_instagram', anonymous=False)

	# Create the NodeName object
	node = live_instagram()

	# Setup proper shutdown behavior
	rospy.on_shutdown(node.on_shutdown)

	# Keep it spinning to keep the node alive
	rospy.spin()
