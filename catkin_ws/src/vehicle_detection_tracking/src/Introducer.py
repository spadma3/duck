#!usr/bin/env python

import cv2
import os
import numpy as np 
import struct, array
import Detector
from subprocess import call

class Introducer():
	def __init__(self,positive,negative):
		self.positive = cv2.imread(positive,0)
		self.negative = cv2.imread(negative,0)
		self.Detector = Detector.Detector()

	def generate_samples(self):
		call("./create_samples.sh")

	def equalize(self):
		self.positive = cv2.equalizeHist(self.positive)
		self.negative = cv2.equalizeHist(self.negative)

	# def offline_train(self):
	# 	self.Detector.train_classifier()

	def offline_train(self,width=36, height=43):
		es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		f = open("./samples.vec",'rb')
		HEADERTYP = '<iihh'
		samplecount,samplesize,_,_ = struct.unpack(HEADERTYP,f.read(12))
		for i in range(samplecount):
			sample = np.zeros((height,width),np.uint8)
			f.read(1)
			data = array.array('h')
			data.fromfile(f,samplesize)
			for r in range(height):
				for c in range(width):
					sample[r,c] = data[r*width+c]
			thresh = cv2.threshold(sample.copy(), 20, 255, cv2.THRESH_BINARY)[1]
			dilate = cv2.dilate(thresh.copy(), es, iterations=1)
			self.Detector.train_classifier([0,0,width,height],dilate,1)
			cv2.imshow('vec_img',dilate)
			cv2.imshow("thresh",thresh)
			k = 0xFF & cv2.waitKey(0)
			if k == 27:         # esc to exit
				break
		pos, neg = self.Detector.get_posterior()
		file_pos = "/home/ubuntu/TLD/posDist"
		file_neg = "/home/ubuntu/TLD/negDist"
		np.save(file_pos,pos)
		np.save(file_neg,neg)

	def run(self):
		# self.equalize()
		# cv2.imwrite("negative.jpg", self.negative)
		# cv2.imwrite("positive.jpg", self.positive)
		self.generate_samples()



introduce = Introducer("positive.png", "negative.png")
introduce.offline_train()
# img = cv2.imread("./positive.jpg",0)
# cv2.imshow("Positive", img)
# cv2.waitKey(0)
