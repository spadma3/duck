#!usr/bin/env python
import cv2
import numpy as np 
from time import time


feature_params = dict( maxCorners=500, qualityLevel=0.3,minDistance=7,blockSize=7)
lk_params = dict( winSize=(10,10), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,10,0.03))

class Tracker2():
	def __init__(self, video_path):
		self.bounding_box = [0]*4
		self.creating_bounding_box = False
		self.bounding_box_created = False
		self.detect_interval = 5
		self.frame_id = 0
		self.video = cv2.VideoCapture(video_path)
		self.viz = None
		self.gray = None
		self.track_points = []
		self.track_len = 10
		cv2.namedWindow("Tracking")
		self.start()


	def start(self):
		_,self.viz = self.video.read()
		cv2.imshow("Tracking", self.viz)
		cv2.setMouseCallback("Tracking", self.create_bounding_box)
		while True:
			if self.creating_bounding_box:
				print "Creating bounding Box"
				cv2.imshow("Tracking", self.viz)
				cv2.waitKey(30)
				continue

			if self.bounding_box_created:
				print "Created Bounding Box, Tracking Begins"
				print self.bounding_box
				self.tracking()

			if not self.bounding_box_created:
				print "No bounding Box selected"
				_,self.viz = self.video.read()
				cv2.imshow("Tracking", self.viz)
				cv2.waitKey(30)
		print "End of start function"

	def create_bounding_box(self,event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.bounding_box[0] = x
			self.bounding_box[1] = y
			creating_bounding_box = True

		elif event == cv2.EVENT_LBUTTONUP:
			self.bounding_box[2] = x
			self.bounding_box[3] = y
			self.bounding_box_created = True
			self.creating_bounding_box = False


	def goodFeature2Track(self):
		self.mask = np.zeros_like(self.gray)
		self.mask[self.bounding_box[1]:self.bounding_box[3],self.bounding_box[0]:self.bounding_box[2]] = 255
		goodFeatures = cv2.goodFeaturesToTrack(self.gray, mask=self.mask,**feature_params)
		if goodFeatures is not None:
			for x,y in np.float32(goodFeatures).reshape(-1,2):
				self.track_points.append([(x,y)])

	def tracking(self):
		while True:
			ret, frame = self.video.read()
			self.gray = cv2.cvtColor(self.viz, cv2.COLOR_BGR2GRAY)
			self.viz = frame.copy()

			if len(self.track_points) > 0:
				p0 = np.float32([tr[-1] for tr in self.track_points]).reshape(-1,1,2)
				p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray,self.gray,p0,None,**lk_params)
				p0r, st, err = cv2.calcOpticalFlowPyrLK(self.gray, self.prev_gray,p1,None,**lk_params)
				d = abs(p0-p0r).reshape(-1,2).max(-1)
				good = d < 1
				new_tracks = []
				for tr, (x,y), good_flag in zip(self.track_points, p1.reshape(-1,2), good):
					if not good_flag:
						continue
					tr.append((x,y))
					if len(tr) > self.track_len:
						del tr[0]
					new_tracks.append(tr)
					cv2.circle(self.viz, (x,y),2,(0,255,0),-1)
				self.track_points = new_tracks
				cv2.polylines(self.viz,[np.int32(tr) for tr in self.track_points],False,(0,255,0))
				print "After each frame", self.track_points
				print

			if self.frame_id % self.detect_interval == 0:
				for x,y in [np.int32(tr[-1]) for tr in self.track_points]:
					cv2.circle(self.mask, (x,y),5,0,-1)
				self.goodFeature2Track()
				print "New features %" %(self.frame_id), self.track_points
				print


			self.frame_id += 1
			self.prev_gray = self.gray
			cv2.imshow("Tracking", self.viz)

			ch = 0xFF & cv2.waitKey(1)
			if ch == 27:
				break

def main(video_src):
	Tracker2(video_src)

if __name__=='__main__':
	main(0)