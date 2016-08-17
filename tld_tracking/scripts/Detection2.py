#!usr/bin/env/python

import cv2
import numpy as np 
from time import time
from time import sleep
import random
import math
import itertools

class Detector():
	"""docstring for Detector"""
	def __init__(self, numFeatures):
		self.samples = 0
		self.img = None
		self.numFeatures = numFeatures
		self.totalFeatures = numFeatures**2
		self.featurePoints = []
		self.variance_detection = 0.0
		# self.posteriors_pos = [0]*2**self.totalFeatures
		# self.posteriors_neg = [0]*2**self.totalFeatures
		# self.classifiers = []
		self.numClassifiers = 4
		self.init_windows = []
		self.windows = []
		self.numWindows = 0
		self.obj2detect = None
		# self.hor_classifiers = [(0,1),(2,3)]
		# self.ver_classifiers = [(0,2),(1,3)]
		self.classifiers = [(0,1),(2,3),(0,2),(1,3)]
		self.disp_img = None
		# cv2.namedWindow("Transformed Patch", cv2.WINDOW_NORMAL)
		self.init_random_points()
		# self.init_posterior_base_class()
		self.init_fern()
		self.sliding_windows()
		print len(self.init_windows)/4
		# self.gen_base_classifier(self.numClassifiers)

	def sliding_windows(self):
		t0 = time()
		scale_factor = 1.2
		min_window_width = 40
		min_window_height = 40
		img_height = 480
		img_width = 640
		win_size_w = min_window_width
		win_size_h = min_window_height
		shift_w = int(round(0.1*win_size_w))
		shift_h = int(round(0.1*win_size_h))
		index = 0
		while win_size_w < img_width/4 and win_size_h < img_height/4:
			for x in xrange(0, img_width-win_size_w, shift_w):
				for y in xrange(0, img_height-win_size_h, shift_h):
					self.init_windows[index:index+4] = [x,y,x+win_size_w, y+win_size_h]
					index +=4
			win_size_h=int(round(scale_factor*win_size_h))
			win_size_w=int(round(scale_factor*win_size_w))
			shift_w = int(round(0.1*win_size_w))
			shift_h = int(round(0.1*win_size_h))
			# print win_size_h, win_size_w
		self.numWindows = len(self.init_windows)/4
		return self.init_windows
	def get_windows(self):
		return self.init_windows

	def show_images(self, frame):
		for i in range(self.numWindows):
			index = 4*i 
			patch = self.windows[index:index+4]
			img = frame[patch[1]:patch[3], patch[0]:patch[2]]
			cv2.imshow("Test", img)
			cv2.waitKey(1)

	def gaussian_filter(self, image, kernel, std):
		self.img = cv2.GaussianBlur(image, kernel, std)

	def cal_integral_img(self):
		integral_img, integral_img_square = cv2.integral2(self.img) # dimensions are w+1 x h+1 remember
		return integral_img, integral_img_square

	# def cal_variance_patch(self):
	# 	variances = [0.0]*self.numWindows
	# 	for i in range(self.numWindows):
	# 		index = 4*i 
	# 		win = self.windows[index:index+4]
	# 		print win 
	# 		win_pixels = (win[2]-win[0]+1)*(win[3]-win[1]+1)
	# 		_sum = integral_img[win[0],win[1]] + integral_img[win[2]+1,win[3]+1] - \
	# 			integral_img[win[0],win[3]+1] - integral_img[win[2]+1,win[1]]
	# 		square_sum = integral_img_sq[win[0],win[1]] + integral_img_sq[win[2]+1,win[3]+1] - \
	# 			integral_img_sq[win[0],win[3]+1] - integral_img_sq[win[2]+1,win[1]]
	# 		mean = _sum/win_pixels
	# 		mean_sq = square_sum/win_pixels
	# 		variances[i] = mean_sq - mean**2
	# 	return variances


	def variance_filter(self):
		integral_img, integral_img_sq = self.cal_integral_img()
		variances = [0.0]*(len(self.windows)/4)
		for i in range(len(self.windows)/4):
			index = 4*i 
			win = self.windows[index:index+4]
			win_pixels = (win[2]-win[0]+1)*(win[3]-win[1]+1)
			_sum = integral_img[win[1],win[0]] + integral_img[win[3]+1,win[2]+1] - \
				integral_img[win[3]+1,win[0]] - integral_img[win[1],win[2]+1]
			square_sum = integral_img_sq[win[1],win[0]] + integral_img_sq[win[3]+1,win[2]+1] - \
				integral_img_sq[win[3]+1,win[0]] - integral_img_sq[win[1],win[2]+1]
			mean = float(_sum)/win_pixels
			mean_sq = float(square_sum)/win_pixels
			variances[i] = mean_sq - mean**2
		

		filter_variance = 1.2*self.variance_detection
		windows_indices = [variances.index(x) for x in variances if x > filter_variance]
		# print windows_indices
		temp = [0]*len(windows_indices)*4
		for i in range(len(windows_indices)):
			j = windows_indices[i]
			ind = 4*j
			i = 4*i 
			temp[i:i+4] = self.windows[ind:ind+4]
		self.windows = temp
		self.numWindows = len(self.windows)/4
		return self.windows   

	def vehicle_variance(self, patch,label):
		if label == 1:
			integral_img, integral_img_sq = self.cal_integral_img()
			win_pixels = (patch[2]-patch[0]+1)*(patch[3]-patch[1]+1)
			# print win_pixels
			win = patch
			_sum = integral_img[win[1],win[0]] + integral_img[win[3]+1,win[2]+1] - \
				integral_img[win[3]+1,win[0]] - integral_img[win[1],win[2]+1]
			square_sum = integral_img_sq[win[1],win[0]] + integral_img_sq[win[3]+1,win[2]+1] - \
				integral_img_sq[win[3]+1,win[0]] - integral_img_sq[win[1],win[2]+1]
			# print _sum, square_sum
			mean = float(_sum)/win_pixels
			mean_sq = float(square_sum)/win_pixels
			# print mean, mean_sq
			variance_detect = mean_sq - mean**2
			# print "New Frame variance: ", variance_detect
			self.variance_detection = (self.variance_detection + variance_detect)/2.0
		# print "Average Variance: ", self.variance_detection

	def init_random_points(self):
		width = 10
		height = 10
		step = 1.0/(self.numFeatures+1)
		quadrants = [0,0,1,0,0,1,1,1]  # Space divieded in to four quadrants in rows
		numQuadrants = 4

		# Uniform Distribution ##
		for quad in xrange(len(quadrants)/2):
			quad = 2*quad
			for i in range(self.numFeatures+1):
				for j in range(self.numFeatures+1):
					if i != 0 and j!= 0:
						self.featurePoints.append(int(round(j*step+quadrants[quad],2)*width))
						self.featurePoints.append(int(round(i*step+quadrants[quad+1],2)*height))
		# Random Distribution ##
		# index = 0
		# self.featurePoints = [i/1000.0 for i in random.sample(range(50,950), self.totalFeatures*2*numQuadrants)]
		# for quad in xrange(len(quadrants)/2):
		# 	quad = 2*quad 
		# 	for i in range(self.totalFeatures):
		# 		i = 2*i
		# 		self.featurePoints[i+index] = round(self.featurePoints[i+index]*width + quadrants[quad]*width,2)
		# 		self.featurePoints[i+1+index] = round(self.featurePoints[i+1+index]*height + quadrants[quad+1]*height,2)
		# 	index += self.totalFeatures*2
		# print len(self.featurePoints)

		## Uniform Distribution with classifier grouping ##
		# for i in range(self.totalFeatures):
		# 	i = i*self.totalFeatures
		# 	for j in range(2):
		# 		self.featurePoints.append(step + quadrants[2*j]*(width-step)*2)
		# 		self.featurePoints.append(step + quadrants[2*j+1]*(height-step)*2)
		# 	step += step
		# print "Feature Points in (2,2) window"
		print self.featurePoints
		return self.featurePoints


	def generate_pixel_points(self,patch):  #ADD SCALES IN WINDOWS DATA STRUCTURE
		featurePoints_patch = self.featurePoints[:]

		# width = (patch[2] - patch[0])/2
		# height = (patch[3] - patch[1])/2
		width = 10
		height = 10

		for i in range(len(self.featurePoints)):
			if i%2 == 0:
				# featurePoints_patch[i] = patch[0]+int(featurePoints_patch[i]*width)
				featurePoints_patch[i] = int(featurePoints_patch[i]*width)
			else:
				# featurePoints_patch[i] = patch[1]+int(featurePoints_patch[i]*height)
				featurePoints_patch[i] = int(featurePoints_patch[i]*height)
		return featurePoints_patch

	def gen_base_classifier(self,numClassifier):
		self.classifiers = list(itertools.combinations(range(numClassifier),2))
		return self.classifiers

	def bool2binary(self,boolean):
		return ''.join(['1' if x else '0' for x in boolean])

	def int2binary(self,integer):
		return '{0:08b}'.format(integer)


	def patch_comparison(self, patch, label):
		# self.samples+=1
		# print patch
		img = self.img[patch[1]:patch[3], patch[0]:patch[2]]
		img = cv2.resize(img, (20,20), interpolation = cv2.INTER_LINEAR)
		# cv2.imwrite('/home/ubuntu/temp/samples/pic{:>04}.jpg'.format(self.samples), img)
		img = cv2.threshold(img,10,255,cv2.THRESH_BINARY)[1]
		# self.disp_img = img
		# print img
		# if label:
		# 	cv2.namedWindow("Threshold Images", cv2.WINDOW_NORMAL)
		# 	cv2.namedWindow("Scaled Images", cv2.WINDOW_NORMAL)
		# 	cv2.imshow("Scaled Images", img_temp)
		# 	cv2.imshow("Threshold Images", img)
		# 	cv2.waitKey(5)
		# print img[0,0]
		# points = self.generate_pixel_points(patch)
		points = self.featurePoints
		# print points
		# print len(points)
		# print self.classifiers
		featureVector = []   # six features from each classifier
		## Similar index comparison ##

		for classifier in self.classifiers:
			boolean = [None]*self.totalFeatures
			ind1 = 2*self.totalFeatures*classifier[0]
			ind2 = 2*self.totalFeatures*classifier[1] + 18
			# print ind1, ind2
			for i in range(self.totalFeatures):
				i = 2*i 
				# boolean[i/2] = abs(self.img[points[i+1+ind1],points[i+ind1]]-self.img[points[i+1+ind2],points[i+ind2]]) <= 5
				# boolean[i/2] = abs(img[points[i+1+ind1],points[i+ind1]]-img[points[ind2-(i+1)],points[ind2-(i+2)]]) <= 10
				boolean[i/2] = img[points[i+1+ind1],points[i+ind1]]==img[points[ind2-(i+1)],points[ind2-(i+2)]]

			# print boolean
			featureVector.append(self.bool2binary(boolean))

		## Symmetric Comparison ##
		# for classifier in self.hor_classifiers:
		# 	boolean = []
		# 	ind1 = classifier[0]
		# 	ind2 = classifier[1]
		# 	for row in range(self.numFeatures):
		# 		start_row1 = row*self.numFeatures*2 + ind1*self.totalFeatures*2
		# 		start_row2 = (row+1)*self.numFeatures*2 + ind2*self.totalFeatures*2
		# 		for col in range(self.numFeatures):
		# 			col_temp = 2*col
		# 			point1 = points[start_row1 + col_temp], points[start_row1 + col_temp+1]
		# 			point2 = points[-len(points)+start_row2-1-(col_temp+1)], points[-len(points)+start_row2-1-col_temp]
		# 			comparison = img[point1] == img[point2]
		# 			boolean.append(comparison)
		# 	featureVector.append(self.bool2binary(boolean))

		# for classifier in self.ver_classifiers:
		# 	boolean = []
		# 	ind1 = classifier[0]
		# 	ind2 = classifier[1]
		# 	for row in range(self.numFeatures):
		# 		start_row1 = ind1*self.totalFeatures*2 + row*self.numFeatures*2
		# 		start_row2 = (ind2+1)*self.totalFeatures*2 - (row+1)*self.numFeatures*2
		# 		for col in range(self.numFeatures):
		# 			col_temp = 2*col
		# 			point1 = points[start_row1 + col_temp], points[start_row1 + col_temp+1]
		# 			point2 = points[start_row2 + col_temp], points[start_row2 + col_temp+1]
		# 			comparison = img[point1] == img[point2]
		# 			boolean.append(comparison)
		# 			# print boolean
		# 	featureVector.append(self.bool2binary(boolean))
		featureVector.append(label)
		return featureVector


	# def update_posterior(self,feature, label):
	# 	if label == 1:
	# 		self.posteriors_pos[int(feature,2)]+=1
	# 	elif label == 0:
	# 		self.posteriors_neg[int(feature,2)]+=1
	# 	else:
	# 		print "Unknown label data type"

	def update_posterior(self, featureVector):
		# print featureVector
		if featureVector[-1] == 1:
			for classifier in range(self.numClassifiers):
				ind = classifier*2**self.totalFeatures
				self.posteriors_pos[ind+int(featureVector[classifier],2)]+=1
		else:
			for classifier in range(self.numClassifiers):
				ind = classifier*2**self.totalFeatures
				self.posteriors_neg[ind+int(featureVector[classifier],2)]+=1

	# def cal_prob(self,featureVector):
	# 	list_prob = []
	# 	for classifier in range(self.numClassifiers):
	# 		ind = classifier*2**self.totalFeatures
	# 		pos = self.posteriors_pos[ind + int(featureVector[classifier],2)]
	# 		neg = self.posteriors_neg[ind + int(featureVector[classifier],2)]
	# 		prob = float(pos)/(pos+neg)
	# 		list_prob.append(prob)
	# 	return list_prob

	def cal_prob(self,featureVector):
		list_prob = []
		list_feat_pos = []
		list_feat_neg = []
		for classifier in range(10):
			ind = classifier*2**10
			pos = self.posteriors_pos[ind + int(featureVector[classifier],2)]
			neg = self.posteriors_neg[ind + int(featureVector[classifier],2)]
			if pos == 0.0 and neg == 0.0:
				prob = 0.25
				list_prob.append(prob)
				list_feat_pos.append(pos)
				list_feat_neg.append(neg)
			else:
				prob = round(float(pos)/(pos+neg),3)
				# print round(pos,3), round(neg,3), prob
				list_prob.append(prob)
				list_feat_pos.append(pos)
				list_feat_neg.append(neg)

		# print list_prob
		return list_prob, list_feat_pos, list_feat_neg

	def isVehicle(self, prob):
		sum_prob = sum(prob)
		average_prob = float(sum_prob)/len(prob)
		if average_prob > 0.5:
			veh = True
			# print prob
		else:
			veh = False
		return round(average_prob,3), veh

	def train_classifiers(self,image,patch,label):
		self.gaussian_filter(image, (3,3),3.0)
		self.vehicle_variance(patch,label)
		featureVector = self.patch_comparison(patch,label)
		self.update_posterior(featureVector)

	def init_fern(self):
		self.posteriors_pos = [1]*(2**10)*10
		self.posteriors_neg = [1]*(2**10)*10

	def train_classifier2(self, image, patch, label):
		self.img = image
		featureVector = self.fern(patch, label)
		self.update_fern(featureVector)

	def fern(self, patch, label):
		win = patch
		height = win[3] - win[1]
		width = win[2] - win[0]
		step_w = width/10
		step_h = height/10
		featureVector = [0]*10
		# print float(np.sum(self.img[win[1]:win[3], win[0]:win[2]]))/((win[2]-win[0])*(win[3]-win[1]))
		for i in range(10):
			boolean = []
			for j in range(10):
				sec = [win[0]+j*step_w,win[1]+i*step_h,win[0]+(j+1)*step_w,win[1]+(i+1)*step_h]
				_sum = np.sum(self.img[sec[1]:sec[3], sec[0]:sec[2]])
				avg = int(round(float(_sum)/(step_h*step_w),2))
				# print "Average od the sub-patch", avg
				if avg < 127:
					# temp_img[sec[1]:sec[3], sec[0]:sec[2]] = 0
					boolean.append(False)
				else:
					# temp_img[sec[1]:sec[3], sec[0]:sec[2]] = 255
					boolean.append(True)
			featureVector[i] = self.bool2binary(boolean)
		featureVector.append(label)
		return featureVector

	def fern2(self,patch,label):
 		win = patch
		img2 = self.img[win[1]:win[3],win[0]:win[2]]
		# img3 = img2.copy()
		h = win[3] - win[1]
		w = win[2] - win[0]
		step_w = w/10
		step_h = h/10
		
		# mode = stats.mode(img2, axis=None)
		# std = np.std(img2)
		mean = np.mean(img2)
		featureVector = [0]*10
		for i in range(10):
			boolean = []
			for j in range(10):
				p = [j*step_w,i*step_h,(j+1)*step_w, (i+1)*step_h]
				_sum = np.sum(img2[p[1]:p[3],p[0]:p[2]])
				avg = int(round(float(_sum)/(step_w*step_h)))
				if avg <= mean:
					# img3[p[1]:p[3],p[0]:p[2]] = 0
					boolean.append(False)
				else:
					# img3[p[1]:p[3],p[0]:p[2]] = 255
					boolean.append(True)
			featureVector[i] = self.bool2binary(boolean)
		featureVector.append(label)
		if label == 1:
			print featureVector
		return featureVector


	def update_fern(self, featureVector):
		if featureVector[-1] == 1:
			for classifier in range(10):
				ind = classifier*2**10
				self.posteriors_pos[ind+int(featureVector[classifier],2)]+=1
		else:
			for classifier in range(10):
				ind = classifier*2**10
				self.posteriors_neg[ind+int(featureVector[classifier],2)]+=1


	def scaling(self):
		sum_pos = sum(self.posteriors_pos[0:2**10]) - (2**10)
		sum_neg = sum(self.posteriors_neg[0:2**10]) - (2**10)
		print sum_pos, sum_neg
		if sum_neg != 0:
			scale = float(sum_pos)/sum_neg
			print "Scale", scale
			for classifier in range(10):
				ind = classifier*2**10
				for i in range(2**10):
					elem = self.posteriors_neg[ind+i]
					if elem != 1:
						if int(round(elem*scale)) > 1:
							self.posteriors_neg[ind+i] = int(round(elem*scale))
							print elem, self.posteriors_neg[ind+i]
						else:
							self.posteriors_neg[ind+i] = 1

	def test_classifier2(self, image):
		param = dict(winSize_match=10, method=cv2.cv.CV_TM_CCOEFF_NORMED)
		self.img = image
		self.windows = self.init_windows[:]
		list_vehicle = []
		list_prob = []
		num = 0
		for i in range(len(self.windows)/4):
			index = 4*i
			win = self.windows[index:index+4]
			val = float(np.sum(self.img[win[1]:win[3], win[0]:win[2]]))/((win[2]-win[0])*(win[3]-win[1]))
			good_patch = val < 190 and val > 120
			# val = np.var(self.img[win[1]:win[3], win[0]:win[2]])
			# good_patch = val > 2500
			if not good_patch:
				continue
			else:
				num+=1
				featureVector = self.fern(win,-1)  #### Note which fern has been used!!!
				prob, pos, neg = self.cal_prob(featureVector)
				av_prob, veh = self.isVehicle(prob)
				# list_prob.append(av_prob)
				if veh:
					list_vehicle.append(win)
					list_prob.append(av_prob)
					print "Features:", pos
					print 
					print neg

		# list_vehicle = [self.nearest_neighbor(list_vehicle, **param)]
		if len(list_prob) > 0:
			# ind = list_prob.index(max(list_prob))
			# print ind
			# list_vehicle2 = [list_vehicle[ind]]
			# print list_vehicle2
			# print "Probabilities", list_prob
			print "Detections", list_vehicle
			print "Initial windows: %i" %(len(self.windows)/4), "windows"
			print "Filtered %i" %(num), "windows"
			print "Detected %i" %(len(list_vehicle)), "windows"
			return list_vehicle
		else:
			return []

	def normalise_hist(self):
		# for i in range(10):
		# 	ind = 2**10
		# 	_sum = sum(self.posteriors_pos[i*ind:(i+1)*ind])
		# 	print _sum
		# 	# print self.posteriors_pos[i*ind:(i+1)*ind]
		# 	for j in range(2**10):
		# 		self.posteriors_pos[i*ind+j] = float(self.posteriors_pos[i*ind+j])/_sum

		# for i in range(10):
		# 	ind = 2**10
		# 	_sum = sum(self.posteriors_neg[i*ind:(i+1)*ind])
		# 	print _sum
		# 	for j in range(2**10):
		# 		self.posteriors_neg[i*ind+j] = float(self.posteriors_neg[i*ind+j])/_sum

		for i in range(10):
			ind = 2**10
			_sum = sum(self.posteriors_pos[i*ind:(i+1)*ind]) - 2**10
			print _sum
			for j in range(2**10):
				self.posteriors_pos[i*ind+j] = float(self.posteriors_pos[i*ind+j]-1)/_sum


		for i in range(10):
			ind = 2**10
			_sum = sum(self.posteriors_neg[i*ind:(i+1)*ind]) - 2**10
			print _sum
			for j in range(2**10):
				self.posteriors_neg[i*ind+j] = float(self.posteriors_neg[i*ind+j]-1)/_sum

	def test_classifiers(self, image):
		self.windows = self.init_windows[:]
		# self.show_images(image)
		self.gaussian_filter(image, (3,3),3.0)
		print "Initial: ", len(self.windows)/4
		self.variance_filter()
		print "After Variance Filter: ", len(self.windows)/4
		# print self.windows
		list_vehicle = []
		list_prob = []
		for i in range(self.numWindows):
			index = 4*i
			win = self.windows[index:index+4]
			# print win
			featureVector = self.patch_comparison(win,-1)
			prob = self.cal_prob(featureVector)
			av_prob, veh = self.isVehicle(prob)
			# print av_prob
			list_prob.append(av_prob)
			if veh:
				list_vehicle.append(win)
		# print list_vehicle
		return list_vehicle

	def nearest_neighbor(self, list_vehicle, winSize_match, method):
		correlations = []
		for veh in list_vehicle:
			image = self.img[veh[1]:veh[3],veh[0]:veh[2]]
			image = cv2.resize(image, (32,32), interpolation=cv2.INTER_NEAREST)
			ncc = []
			for i in range(22,152,1):
				template = cv2.imread("/home/ubuntu/temp/positives/pic{:>03}.jpg".format(i),0)
				# print template.shape
				# cv2.imshow("Template", template)
				# cv2.waitKey(1000)
				template = cv2.resize(template, (32,32), interpolation=cv2.INTER_NEAREST)
				matching = cv2.matchTemplate(image, template, method)[0][0]
				ncc.append(matching)
			max_ncc = max(ncc)
			correlations.append(max_ncc)
		max_correlation = max(correlations)
		return list_vehicle[correlations.index(max_correlation)]

	def get_posterior(self):
		return self.posteriors_pos, self.posteriors_neg


	def init_posterior_base_class(self):
		self.posteriors_pos = [1]*2**self.totalFeatures*self.numClassifiers
		self.posteriors_neg = [1]*2**self.totalFeatures*self.numClassifiers


	def generate_negative(self,image, bbox):
		box = [bbox[0]-50, bbox[1]-50,bbox[2],bbox[3]]
		while True:
			point = [random.randint(0,640), random.randint(0,480)]
			win = [point[0], point[1], point[0]+50,point[1]+50]
			p1Outside = point[0]>box[2] or point[0]<box[0] or point[1]>box[3] or point[1]<box[1]
			p2Inside = win[2]<640 and win[3]<480
			if p1Outside and p2Inside:
				val = float(np.sum(self.img[win[1]:win[3], win[0]:win[2]]))/((win[2]-win[0])*(win[3]-win[1]))
				patch2train = val < 210.0 and val > 45.0
				self.train_classifier2(image, win,0)
				break
			else:
				continue
		# print "Bounding Box", bbox
		# num  = 0
		# for i in range(len(self.init_windows)/4):
		# 	ind = 4*i 
		# 	box = self.init_windows[ind:ind+4]
		# 	width = box[2] - box[0]
		# 	height = box[3] - box[1]
		# 	bbox_new = [bbox[0] - width, bbox[1]-height, bbox[2], bbox[3]]
		# 	outside = box[0]>bbox_new[2] or box[0]<bbox_new[0] or box[1]>bbox_new[3] or box[1]<bbox_new[1]
		# 	if outside:
		# 		val = float(np.sum(self.img[box[1]:box[3], box[0]:box[2]]))/((box[2]-box[0])*(box[3]-box[1]))
		# 		patch2train = val < 210.0 and val > 45.0
		# 		if not patch2train:
		# 			continue
		# 		else:
		# 			num+=1
		# 			self.train_classifier2(image, box,0)

		# 	else:
		# 		continue
		# print "Total number of negatives: %i" %(num) , "windows"
		# for i in range(len(self.init_windows)/4):
		# 	ind = 4*i 
		# 	# print len(self.init_windows), ind
		# 	box = self.init_windows[ind:ind+4]
		# 	# print "Training box", box
		# 	if abs(box[0]-bbox[0])>10 and abs(box[1]-bbox[1])>10 and abs(box[2]-bbox[2])>10 and abs(box[3]-bbox[3])>10:
		# 		self.train_classifiers(image,box,0)
		# 	else:
		# 		continue

	def generate_negative2(self, image, bbox):
		num  = 0
		for i in range(len(self.init_windows)/4):
			ind = 4*i 
			box = self.init_windows[ind:ind+4]
			width = box[2] - box[0]
			height = box[3] - box[1]
			bbox_new = [bbox[0] - width, bbox[1]-height, bbox[2], bbox[3]]
			outside = box[0]>bbox_new[2] or box[0]<bbox_new[0] or box[1]>bbox_new[3] or box[1]<bbox_new[1]
			if outside:
				val = np.var(self.img[box[1]:box[3], box[0]:box[2]])
				patch2train = val > 2500
				if not patch2train:
					continue
				else:
					num+=1
					self.train_classifier2(image, box,0)

			else:
				continue
		print "Total number of negatives: %i" %(num) , "windows"


import numpy as np
import matplotlib.pyplot as plt

FIG = 0

def plot(x):
	global FIG
	for i in range(10):
		fig = plt.figure(FIG)
		ind = 2**10
		y = x[i*ind:(i+1)*ind]
		print len(y)
		# plt.hist(x)
		pos = np.arange(len(y))
		width = 1.0     # gives histogram aspect to the bar diagram

		ax = plt.axes()
		ax.set_xticks(pos + (width / 2))
		ax.set_xticklabels(range(len(y)))

		plt.bar(pos, y, width, color='r')
		FIG+=1


def readFrames():
	global FIG
	bs = cv2.BackgroundSubtractorMOG()
	cv2.namedWindow("detection", cv2.WINDOW_NORMAL)
	# cv2.namedWindow("Detected Window", cv2.WINDOW_NORMAL)
	cv2.namedWindow("Detection_Test", cv2.WINDOW_NORMAL)

	video_feed1 = cv2.VideoCapture("/home/ubuntu/temp/video_test/background/background.mpg")
	video_feed2 = cv2.VideoCapture("/home/ubuntu/temp/video_test/vehicle/vehicle2.mpg")
	video_feed3 = cv2.VideoCapture("/home/ubuntu/temp/testing_data/vehicle/vehicle.mpg")
	# video_feed4 = cv2.VideoCapture("/home/ubuntu/temp/testing_data/background/background.mpg")
	# video_feed3.set(cv2.cv.CV_CAP_PROP_POS_FRAMES,10)
	es = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
	t0 = time()
	norm_scale = False
	train = True
	while(True):

		if (time() - t0) < 8:
			_, frame = video_feed1.read()
			fg_mask = bs.apply(frame, learningRate=0.9)
			cv2.waitKey(100)
			continue

		if train:
			_, frame = video_feed2.read()
			frameNo = video_feed2.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
			if  frameNo > 30:
				train = False

			# if frame == None:
			# 	print "No frames"
			# 	train = False
			# 	continue
				# # test_detect.scaling()
				# test_detect.normalise_hist()
				# pos, neg = test_detect.get_posterior()
				# plot(pos)
				# plot(neg)
				# plt.show('hold')
				# break
			fg_mask = bs.apply(frame, learningRate=0)
			th = cv2.threshold(fg_mask.copy(), 244, 255, cv2.THRESH_BINARY)[1]
			dilated = cv2.dilate(th, es, iterations=2)
			contours, hier = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			for c in contours:
				if cv2.contourArea(c) > 800:
					(x,y,w,h) = cv2.boundingRect(c)
					box = [x+2,y+1,x+w-2,y+h-3]
					image_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
					img = cv2.GaussianBlur(image_gray, (3,3), 3.0)
					img2 = cv2.equalizeHist(img)
					img3 = cv2.threshold(img2, 10, 255, cv2.THRESH_BINARY)[1]
					img4 = cv2.erode(img3, es, iterations=2)
					# positive = img4[box[1]:box[3],box[0]:box[2]]
					# patch = img[box[1]:box[3],box[0]:box[2]]
					test_detect.train_classifier2(img4, box, 1)
					# cv2.imwrite('/home/ubuntu/temp/positives/pic{:>03}.jpg'.format(int(frameNo)), positive)
					# cv2.imwrite('/home/ubuntu/temp/patches/pic{:>03}.jpg'.format(int(frameNo)), patch)
					cv2.rectangle(frame, (box[0],box[1]), (box[2],box[3]), (0,255,0),1)
					img_show = img[box[1]:box[3],box[0]:box[2]]
					cv2.imshow("Detection_Test", img_show)
					cv2.waitKey(50)
					if frameNo%30 == 0:
						print "Negative Samples"
						test_detect.generate_negative(img4, box)
					# 	test_detect.train_classifiers(image_gray, neg, 0)
					# cv2.rectangle(frame, (neg[0],neg[1]), (neg[2],neg[3]),(0,0,255),1)
					# test_detect.generate_negative(image_gray, box)
				cv2.imshow("detection",frame)
				if cv2.waitKey(30) & 0xff == ord('q'):
					break
		if not train:
			if not norm_scale:
				# test_detect.scaling()
				test_detect.normalise_hist()
				norm_scale = True
			_, frame = video_feed3.read()
			frameNo = video_feed3.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
			if  frameNo == 20:
				pos, neg = test_detect.get_posterior()
				plot(pos)
				plot(neg)
				plt.show('hold')
				break
			image_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			img = cv2.GaussianBlur(image_gray, (3,3), 3.0)
			img2 = cv2.equalizeHist(img)
			img3 = cv2.threshold(img2, 10, 255, cv2.THRESH_BINARY)[1]
			img4 = cv2.erode(img3, es, iterations=2)
			list_veh = test_detect.test_classifier2(img4)
			if len(list_veh) > 0:
				for win in list_veh:
					cv2.rectangle(frame, (win[0],win[1]), (win[2],win[3]),(255,0,0),1)
					img5 = img[win[1]:win[3], win[0]:win[2]]
					cv2.imshow("Detected Window", img5)
					cv2.waitKey(100)
			cv2.imshow("detection",frame)
			cv2.imshow("Filtered Image", img4)
			if cv2.waitKey(30) & 0xff == ord('q'):
				break
			# break
		# break
	video_feed2.release()
	video_feed1.release()
	video_feed3.release()
	# video_feed4.release()
	cv2.destroyAllWindows()
	FIG = 0

test_detect = Detector(4)
readFrames()



