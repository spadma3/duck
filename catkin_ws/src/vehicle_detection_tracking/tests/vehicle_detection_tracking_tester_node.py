#!usr/bin/env python

import cv2
import numpy as np 
import Detector
import Tracker


class TLD():
	def __init__(self,video,pos_dist,neg_dist):
		self.video = cv2.VideoCapture(video)
		self.video.set(cv2.cv.CV_CAP_PROP_POS_FRAMES,32)
		self.Detector = Detector.Detector()
		self.Detector.set_posterior(pos_dist,neg_dist)
		self.Tracker = Tracker.Tracker()
		self.tracking = False


	def run(self):
		while True:
			ret,frame = self.video.read()
			if ret:
				frame = frame[80:200,0:640]
				if not self.tracking:
					print "Start Detecting"
					veh = self.Detector.run(frame)
					if veh == None:
						cv2.imshow("Detection",frame)
						cv2.waitKey(1000)
						continue
					else:
						print "Vehicle Detected"
						cv2.rectangle(frame, (veh[0],veh[1]), (veh[2],veh[3]),(255,0,0),1)
						cv2.imshow("Detection", frame)
						cv2.waitKey(100)
						self.Tracker.initialize(veh,frame)
						self.tracking = True
				else:
					veh = self.Tracker.run(frame)
					if veh == None:
						print "Unable to track vehicle"
						self.tracking = False
						continue
					else:
						print "Vehicle Tracked"
						


pos_dist = np.load("/home/ubuntu/TLD/posDist.npy").tolist()
neg_dist = np.load("/home/ubuntu/TLD/negDist.npy").tolist()
video = "/home/ubuntu/temp/video_test/vehicle/vehicle2.mpg"
# video = "/home/ubuntu/temp/testing_data/vehicle/vehicle.mpg"

TLD = TLD(video,pos_dist,neg_dist)
TLD.run()