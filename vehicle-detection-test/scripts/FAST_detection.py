#! usr/bin/python
import rospy
import cv2
import numpy as numpy
from matplotlib import pyplot as plt

class OpenCV:
	def __int__(self):
		self.img = cv2.imread('')
		self.fast = cv2.FastFeatureDetector()

	def procesImage(self, img):
		self.fast.setBool('nonmaxSuppression',0)
		key_points = self.fast.detect(self.img, None)
		img2 = cv2.drawKeypoints(self.img, key_points, color = (255,0,0))
		cv2.imshow('fast_feature_detection1.jpg', img2)

		self.fast.setBool('nonmaxSuppression',1)
		key_points = self.fast.detect(self.img, None)
		img3 = cv2.drawKeypoints(self.img, key_points, color = (255,0,0))
		cv2.imshow('fast_feature_detection2.jpg', img3)

if __name__ == '__main__':
	main()

