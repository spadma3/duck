#! usr/bin/python
import rospy
import cv2
import numpy as numpy
from matplotlib import pyplot as plt


class OpenCV:
	def __init__(self):
		self.img = cv2.imread('/home/ubuntu/duckietown/vehicle-detection-test/test-images/test1.jpg')
		self.fast = cv2.FastFeatureDetector()

	def procesImage(self):
		self.fast.setBool('nonmaxSuppression',1)
		key_points = self.fast.detect(self.img, None)
		img2 = cv2.drawKeypoints(self.img, key_points, color = (255,0,0))
		cv2.namedWindow('Fast_Corner_Detection1', cv2.WINDOW_NORMAL)
		cv2.imshow('Fast_Corner_Detection1', img2)
		# cv2.waitKey(0)

def display():
	obj = OpenCV()
	obj.procesImage()

if __name__ == '__main__':
	display()

