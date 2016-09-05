#!/usr/bin/env python
import cv2
import numpy as np 
from Detector import Detector 
from Tracker import Tracker
import matplotlib.pyplot as plt 

class Introducer():
	def __init__(self):
		try:
			self.template = cv2.imread("/home/ubuntu/duckietown/catkin_ws/src/vehicle_detection_tracking/ObjectModel/template001.jpg",0)
		except IOError:
			print "Object to detect is unknown."
		self.numWindows = 0
		self.init_windows = []
		self.sliding_windows()
		self.Tracker = Tracker()
		self.Detector = Detector()

	def sliding_windows(self):
		scale_factor = 1.05
		min_window_width = 30
		min_window_height = 36
		img_height = 120
		img_width = 640
		win_size_w = min_window_width
		win_size_h = min_window_height
		shift_w = int(round(0.1*win_size_w))
		shift_h = int(round(0.1*win_size_h))
		index = 0
		while win_size_w < img_width/4 and win_size_h < img_height:
			for x in xrange(0, img_width-win_size_w, shift_w):
				for y in xrange(0, img_height-win_size_h, shift_h):
					self.init_windows[index:index+4] = [x,y,x+win_size_w, y+win_size_h]
					index +=4
					self.numWindows += 1
			win_size_h=int(round(scale_factor*win_size_h))
			win_size_w=int(round(scale_factor*win_size_w))
			shift_w = int(round(0.1*win_size_w))
			shift_h = int(round(0.1*win_size_h))
		print self.numWindows
		return self.init_windows

	def patch_matching(self, image):
		correlation = 0
		vehicle = None
		ind = 0
		while ind < len(self.init_windows):
			win = self.init_windows[ind:ind+4]
			ind += 4
			patch = image[win[1]:win[3],win[0]:win[2]]
			template = cv2.resize(self.template, (win[2]-win[0],win[3]-win[1]),interpolation=cv2.INTER_LINEAR)
			matching = cv2.matchTemplate(patch, template, method=cv2.cv.CV_TM_CCOEFF_NORMED)[0][0]
			if matching > correlation:
				correlation = matching
				vehicle = win 
		return vehicle, correlation

	def run(self,frame):
		image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		# self.show_template()
		vehicle,correlation = self.patch_matching(image)
		self.show_image(image,vehicle)
		print correlation
		if correlation > 0.7:
			self.Detector.train(vehicle,frame,1,0)
			self.Detector.generate_negative(frame,vehicle,10)
		else:
			pass

	def show_image(self,image,veh):
		cv2.rectangle(image,(veh[0],veh[1]),(veh[2],veh[3]),(0,255,0),1)
		cv2.imshow("Introducer", image)
		cv2.waitKey(10)

	def show_template(self):
		cv2.imshow("Template", self.template)
		cv2.waitKey(10)

	def save_distribution(self):
		self.Detector.normalise_hist()
		pos,neg = self.Detector.get_posterior()
		np.save("/home/ubuntu/duckietown/catkin_ws/src/vehicle_detection_tracking/ClassifierDistribution/posDist",pos)
		np.save("/home/ubuntu/duckietown/catkin_ws/src/vehicle_detection_tracking/ClassifierDistribution/negDist",neg)