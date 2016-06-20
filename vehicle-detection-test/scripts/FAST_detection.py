#! usr/bin/python
import rospy
import cv2
import numpy as numpy
from matplotlib import pyplot as plt


class OpenCV:
	def __init__(self):
		self.img = cv2.imread('/home/racecar/duckietown/vehicle-detection-test/test-images/test1.jpg')
		self.fast = cv2.FastFeatureDetector()

	def procesImage(self):
		cv2.imshow('test1', self.img)
		self.fast.setBool('nonmaxSuppression',0)
		key_points = self.fast.detect(self.img, None)
		img2 = cv2.drawKeypoints(self.img, key_points, color = (255,0,0))
		cv2.imwrite('fast_feature_detection1.jpg', img2)

		self.fast.setBool('nonmaxSuppression',1)
		key_points = self.fast.detect(self.img, None)
		img3 = cv2.drawKeypoints(self.img, key_points, color = (255,0,0))
		cv2.imwrite('fast_feature_detection2.jpg', img3)

def display():
	obj = OpenCV()
	obj.procesImage()

if __name__ == '__main__':
	display()

