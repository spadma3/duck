import cv2
import numpy as np
from time import time


class Tracker():
	def __init__(self, video_path=None):
		self.bounding_box = [(30,30), (100,100)]
		self.creating_bounding_box = False
		self.init_pts_density = 4
		self.margin = 2
		self.start_img = None
		self.target_img = None
		self.viz = None
		if not video_path:
			raise SystemExit("Please upload a video to track the object")
		else:
			self.video = cv2.VideoCapture(video_path)
		self.start()
		self.tracking()


	def show_window(self):
		cv2.namedWindow("Tracking")
		while True:
			cv2.imshow("Tracking", self.viz)
			key = cv2.waitKey(1) & 0xFF

			if key == ord("r"):
				image = clone_image.copy()

			if key == ord("c"):
				break

	def start(self):
		cv2.namedWindow("Tracking")
		# cv2.setMouseCallback("Tracking", self.create_bounding_box)
		while True:
			flag, self.viz = self.video.read()
			while True:
				if self.creating_bounding_box:	
					continue
				else:
					cv2.waitKey(30)
					break
			if self.bounding_box:
				self.start_img = cv2.cvtColor(self.viz, cv2.COLOR_BGR2GRAY)
				break

	def tracking(self):
		while True:
			start_bb = self.bounding_box
			flag, self.viz = self.video.read()
			print self.video.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
			self.target_img = cv2.cvtColor(self.viz, cv2.COLOR_BGR2GRAY)
			start_pts = self.gen_point_cloud(start_bb)  #[(x,y),(x,y)...]
			corr, dist, target_pts, start_pts = self.cal_target_pts(start_pts)
			good_start_pts, good_target_pts = self.filter_pts(corr, dist, target_pts, start_pts)
			self.bounding_box = self.target_bounding_box(start_bb,good_start_pts, good_target_pts)
			print self.bounding_box
			cv2.rectangle(self.viz, self.bounding_box[0], self.bounding_box[1], (0,255,0),1)
			self.start_img = self.target_img
			cv2.imshow("Tracking", self.viz)
			cv2.waitKey(10)

	def create_bounding_box(self,event, x, y, flags, param):
		self.show_window()
		if event == cv2.EVENT_LBUTTONDOWN:
			self.bounding_box.append((x,y))
			creating_bounding_box = True

		elif event == cv2.EVENT_LBUTTONUP:
			self.bounding_box.append((x,y))

			self.start_img = cv2.cvtColor(self.viz, cv2.COLOR_BGR2GRAY)
			cv2.rectangle(self.viz, self.bounding_box[0], self.bounding_box[1], (0,255,0),1)
			print self.bounding_box
			self.creating_bounding_box = False
			self.tracking()


	def gen_point_cloud(self, box):
		pts = [] 		# [(x1,y1),(x2,y2)...]
		img_track = [(box[0][0]+self.margin, box[0][1]+self.margin), \
		(box[1][0]-self.margin,box[1][0]-self.margin)]
		numY = ((img_track[1][1] - img_track[0][1])/self.init_pts_density) + 1
		numX = (img_track[1][0] - img_track[0][0])/self.init_pts_density + 1
		for i in range(numX):
			for j in range(numY):
				pts_x = box[0][0] + i*self.init_pts_density
				pts_y = box[0][1] + j*self.init_pts_density
				pts.append((pts_x,pts_y))
		return pts

	def cal_target_pts(self,pts0):
		valid_target_pts = [] # initialize the target points with equal length to source
		valid_start_pts = []
		start_pts = np.asarray(pts0, dtype="float32")
		target_pts = np.asarray(pts0, dtype="float32")
		back_pts = np.asarray(pts0, dtype="float32")
		lk_params = dict(winSize=(15,15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | \
			cv2.TERM_CRITERIA_COUNT,10,0.03),flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
		matching_param = dict(winSize_match=4, method=cv2.cv.CV_TM_CCOEFF_NORMED)

		target_pts, status_forward,_ = cv2.calcOpticalFlowPyrLK(self.start_img,self.target_img,start_pts,target_pts,**lk_params) 
		back_pts, status_backward,_ = cv2.calcOpticalFlowPyrLK(self.start_img,self.target_img,target_pts,back_pts,**lk_params)
		status = status_forward & status_backward

		dist_all = self.euclidean_distance(start_pts, target_pts)
		valid_corr = self.patch_matching(start_pts,target_pts,status,**matching_param) 
		valid_dist = [] 

		for i in np.argwhere(status):
			i = i[0]
			valid_target_pts.append(target_pts[i])
			valid_start_pts.append(start_pts[i])
			valid_dist.append(dist_all[i])

		test = len(valid_start_pts) == len(valid_target_pts) == len(valid_dist) == len(valid_corr)
		return valid_corr, valid_dist, valid_target_pts, valid_start_pts
			
	def patch_matching(self,start_pts,target_pts,status,winSize_match,method):
		match_patches = []
		for i in np.argwhere(status):
			i = i[0]
			patch_start = cv2.getRectSubPix(self.start_img,(winSize_match,winSize_match),tuple(start_pts[i]))
			patch_target = cv2.getRectSubPix(self.target_img,(winSize_match,winSize_match),tuple(target_pts[i]))
			match_patches.append(cv2.matchTemplate(patch_start,patch_target,method)[0][0])
		return match_patches

	def euclidean_distance(self,start_pts,target_pts):
		dist = ((target_pts[:,0]-start_pts[:,0])**2 + (target_pts[:,1]-start_pts[:,1])**2)**0.5
		return dist

	def filter_pts(self,valid_corr, valid_dist, valid_target_pts, valid_start_pts):
		good_target_points = []
		good_start_points = []
		medDist = self.median(valid_dist)
		medCorr = self.median(valid_corr)
		for i in range(len(valid_dist)):
			if valid_dist[i] <= medDist and valid_corr[i] >= medCorr:
				good_target_points.append(valid_target_pts[i])
				good_start_points.append(valid_target_pts[i])
		return good_target_points, good_start_points

	def target_bounding_box(self,start_box,good_start_points, good_target_points):
		num_target_pts = len(good_target_points)
		width_start = start_box[1][0] - start_box[0][0]
		height_start = start_box[1][1] - start_box[0][1]
		diff_x = []
		diff_y = []
		for i in range(num_target_pts):
			diff_x.append(good_target_points[i][0] - good_start_points[i][0])
			diff_y.append(good_target_points[i][1] - good_start_points[i][1])
		dx = self.median(diff_x)
		dy = self.median(diff_y)
		diff_y = diff_x = 0

		scale_factor = []
		for i in range(num_target_pts):
			for j in range(i+1, num_target_pts):
				start_img = ((good_start_points[i][0]-good_start_points[j][0])**2 \
					+ (good_start_points[i][1] - good_start_points[j][1])**2)**0.5
				target_img = ((good_target_points[i][0]-good_target_points[j][0])**2 \
					+ (good_target_points[i][1] - good_target_points[j][1])**2)**0.5
				scale_factor.append(float(target_img)/start_img)
		
		scale = self.median(scale_factor)
		
		if scale == 0:
			scale = 1
		scale_x = ((scale -1)/2)*width_start
		scale_y = ((scale-1)/2)*height_start

		x1_new = start_box[0][0] + dx - scale_x
		x2_new = start_box[1][0] + dx + scale_x
		y1_new = start_box[0][1] + dy - scale_y
		y2_new = start_box[1][1] + dy + scale_y

		target_box = [(int(x1_new),int(y1_new)), (int(x2_new), int(y2_new))]
		return target_box

	def median(self,data):
		new_data = list(data)
		new_data.sort()
		if len(new_data) <1:
			print "No Data point to calculate median"
			return None
		else:
			return new_data[len(new_data)/2]

test_video = Tracker("/home/ubuntu/Original_Images_Bag/run_video/extract_run3/output3.mpg")
test_video.tracking()