#!usr/bin/env/ python
import cv2
import numpy as np
from time import time
import random
import math
# class Detector():
# 	def __init__(self):
# 		self.image_size = (640,480)
# 		self.shift = 0.1
# 		self.scale_factor = 1.2
# 		self.min_win_w = 35
# 		self.min_win_h = 35
# 		self.max_win_w = 350
# 		self.max_win_h = 350


# def show_rectange(w):
# 	cv2.namedWindow("Image")
# 	img = cv2.imread("/home/ubuntu/image.png",0)
# 	img = cv2.GaussianBlur(img, (5,5),3.0)
# 	cv2.imshow("FilterImage", img)
# 	# img4 = img - img3
# 	# cv2.imshow("Difference", img4)
# 	for i in range(len(w)/4):
# 		i = i*4
# 		img2 = img.copy()
# 		cv2.rectangle(img2, (w[i],w[i+1]), (w[i+2],w[i+3]),(0,255,0),2)
# 		cv2.imshow("Image", img2)
# 		cv2.waitKey(5)
# 	cv2.waitKey(0)

# def sliding_windows():
	# t0 = time()
	# scale_factor = 1.2
	# scaling = 10
	# scale = [1.2**x for x in xrange(-scaling, scaling+1, 1)]
	# max_win_w = scale[-1]*(bounding_box[1][0]-bounding_box[0][0])
	# max_win_h = scale[-1]*(bounding_box[1][1]-bounding_box[0][1])
	# shift_w = int(round(0.1*bounding_box))
	# shift_h = int(round(0.1*bounding_box))
	# numWindows = 0

	# while (window_size_w<=max_win_w) and (window_size_h<=max_win_h):
	# 	numWindows += floor(float(240-window_size_w+shift_w)/shift_w)*floor(float(320-window_size_h+shift_h)/shift_h)
	# 	window_size_h=int(round(scale_factor*window_size_h))
	# 	window_size_w=int(round(scale_factor*window_size_w))
	# 	shift_w = int(round(0.1*window_size_w))
	# 	shift_h = int(round(0.1*window_size_h))
	# for sc in range(scale):
	# 	numWindows += floor(float(imgSize[0]-bounding_box+shift_w)/shift_w)*floor(float(imgSize[1]-bounding_box))

	# window_size_w = 10
	# window_size_h = 10
	# shift_w = int(round(0.1*window_size_w))
	# shift_h = int(round(0.1*window_size_h))

	# windows = [0]*(4*int(numWindows))
	# # print windows
	# index = 0
	# while (window_size_w<=max_win_w) and (window_size_h<=max_win_h):
	# 	for x in xrange(0,240-window_size_w+1,shift_w):
	# 		for y in xrange(0, 320-window_size_h+1, shift_h):
	# 			windows[index:index+4] = [x,y,x+window_size_w, y+window_size_h]
	# 			index +=4
	# 	window_size_h=int(round(scale_factor*window_size_h))
	# 	window_size_w=int(round(scale_factor*window_size_w))
	# 	shift_w = int(round(0.1*window_size_w))
	# 	shift_h = int(round(0.1*window_size_h))
	# 	# print window_size_w, shift_w
	# print "Time Taken: %f" %(time()-t0)
	# print len(windows)
	# print numWindows
	# return windows
# 	t0 = time()
# 	scale_factor = 1.2
# 	min_window_width = 30
# 	min_window_height = 30
# 	windows = []
# 	img_height = 480/2
# 	img_width = 640/2
# 	win_size_w = min_window_width
# 	win_size_h = min_window_height
# 	shift_w = int(round(0.1*win_size_w))
# 	shift_h = int(round(0.1*win_size_h))
# 	index = 0
# 	while win_size_w < img_width and win_size_h < img_height:
# 		print shift_w,shift_h,win_size_w, win_size_h
# 		for x in xrange(0, img_width-win_size_w+1, shift_w):
# 			for y in xrange(0, img_height-win_size_h+1, shift_h):
# 				windows[index:index+4] = [x,y,x+win_size_w, y+win_size_h]
# 				index +=4
# 		# print windows
# 		win_size_h=int(round(scale_factor*win_size_h))
# 		win_size_w=int(round(scale_factor*win_size_w))
# 		shift_w = int(round(0.1*win_size_w))
# 		shift_h = int(round(0.1*win_size_h))
# 	print time()-t0
# 	show_rectange(windows)
# 	print len(windows)
# 	return windows

# sliding_windows()
# # test(initWindowandScales())


# def calIntegralImage(img):
# 	# print img
# 	data = np.zeros(img.shape)
# 	# print data
# 	for x in xrange(img.shape[0]):
# 		for y in xrange(img.shape[1]):
# 			if x == 0 and y == 0:
# 				data[x][y] = img[x][y]
# 			elif x == 0:
# 				data[x][y] = img[x][y] + data[x][y-1]
# 			elif y == 0:
# 				data[x][y] = img[x][y] + data[x-1][y]
# 			else:
# 				data[x][y] = data[x][y-1] + data[x-1][y] - data[x-1][y-1] + img[x][y]
# 	# print data
# 	return data
img = np.float32(np.random.randint(5,size = (5,5)))
print img
print img.shape
# t0 = time()
# print calIntegralImage(img)
# print time()-t0


def cal_integral_img(img):
	integral_img, integral_img_square = cv2.integral2(img) # dimensions are w+1 x h+1 remember
	# print integral_img
	# print integral_img_square
	return integral_img, integral_img_square

# t0 = time()
integral_img, integral_img_sq = cal_integral_img(img)
print integral_img
# print time()-t0

windows = [0,0,4, 4, 1, 1,5,5]
numWindows = 2

def cal_variance_patch(integral_img, integral_img_sq, windows, numWindows):
	variances = [0.0]*numWindows
	for i in range(numWindows):
		index = 4*i 
		win = windows[index:index+4]
		print win
		win_pixels = (win[2]-win[0]+1)*(win[3]-win[1]+1)
		print win_pixels
		temp_sum = integral_img[win[0],win[1]] + integral_img[win[2]+1,win[3]+1] - \
			integral_img[win[0],win[3]+1] - integral_img[win[2]+1,win[1]]
		print temp_sum
		square_sum = integral_img_sq[win[0],win[1]] + integral_img_sq[win[2]+1,win[3]+1] - \
			integral_img_sq[win[0],win[3]+1] - integral_img_sq[win[2]+1,win[1]]
		print square_sum
		mean = float(temp_sum)/win_pixels
		mean_sq = float(square_sum)/win_pixels
		print mean, mean_sq
		variances[i] = mean_sq - mean**2
	return variances

print cal_variance_patch(integral_img, integral_img_sq, windows, numWindows)

# def variance_filter(integral_img, integral_img_sq, win_tracking, variances, windows):
# 	win_pixels = (win_tracking[2]-win_tracking[0]+1)*(win_tracking[3]-win_tracking[1]+1)
# 	temp_sum = integral_img[win[0],win[1]] + integral_img[win[2]+1,win[3]+1] - \
# 			integral_img[win[0],win[3]+1] - integral_img[win[2]+1,win[1]]
# 	square_sum = ntegral_img_sq[win[0],win[1]] + integral_img_sq[win[2]+1,win[3]+1] - \
# 			integral_img_sq[win[0],win[3]+1] - integral_img_sq[win[2]+1,win[1]]
# 	mean = temp_sum/win_pixels
# 	mean_sq = square_sum/win_pixels
# 	variance_tracking = mean_sq - mean**2
# 	filter_variance = 0.5*variance_tracking
# 	windows_indices = [variances.index(x) for x in variances if x > filter_variance]
# 	windows = [windows[i] for i in windows_indices]
# 	return windows
# from random import Random
# myrandom = Random()


# def init_random_points():
# 	width = 1
# 	height = 1
# 	numQuadrants = 4
# 	numFeatures = 9
# 	featurePoints = []
# 	step = 1.0/(numFeatures+1)
# 	quadrants = [0,0,1,0,1,1,0,1]
# 	for quad in xrange(len(quadrants)/2):
# 		quad = 2*quad
# 		for i in range(numFeatures+1):
# 			for j in range(numFeatures+1):
# 				if i != 0 and j!= 0:
# 					featurePoints.append(round(j*step+quadrants[quad],2))
# 					featurePoints.append(round(i*step+quadrants[quad+1],2))
# 	# featurePoints_patch = np.asarray(featurePoints)
# 	print featurePoints
# 	return featurePoints

# 	print featurePoints
# 	index = 0

# 	featurePoints = [i/1000.0 for i in random.sample(range(50,950), numFeatures*2*numQuadrants)]
# 	for quad in xrange(len(quadrants)/2):
# 		quad = 2*quad 
# 		for i in range(numFeatures):
# 			i = 2*i
# 			featurePoints[i+index] = round(featurePoints[i+index]*width + quadrants[quad]*width,2)
# 			featurePoints[i+1+index] = round(featurePoints[i+1+index]*height + quadrants[quad+1]*height,2)
# 		index += numFeatures*2
# 	print featurePoints
# 	return featurePoints


# def generate_pixel_points(patch):  #ADD SCALES IN WINDOWS DATA STRUCTURE
# 	featurePoints = init_random_points()
# 	t0 = time()
# 	for p in range(1000):
# 		featurePoints_patch = featurePoints[:]
# 		width = (patch[2] - patch[0])/2
# 		height = (patch[3] - patch[1])/2
# 		for i in range(len(featurePoints)):
# 			if i%2 == 0:
# 				featurePoints_patch[i] = patch[0]+int(featurePoints_patch[i]*width)
# 			else:
# 				featurePoints_patch[i] = patch[1]+int(featurePoints_patch[i]*height)
# 	print time()-t0
# 	print featurePoints_patch
# 	return featurePoints_patch

# patch = [300, 300, 350, 350]
# generate_pixel_points(patch)
# # print time()-t0

# def init_base_classifier():
# 	classifiers = list(itertools.combinations([0,1,2,3],2))
# 	return classifiers

# def patch_comparison(classifiers, patch, img):
# 	featurePoints = generate_pixel_points(patch)
# 	featureVector = []
# 	boolean = []*numFeatures   # initialise it at the beginning
# 	for classifier in classifiers:
# 		ind1 = 2*numFeatures*classifier[0]
# 		ind2 = 2*numFeatures*classifier[1]
# 		for i in range(numFeatures):
# 			i = 2*i 
# 			boolean[i] = img[featurePoints[i+ind1],featurePoints[i+1+ind1]] == img[featurePoints[i+ind2],featurePoints[i+1+ind2]]
# 		featureVector.append(bool2binary(boolean))


# def bool2binary(boolean):
# 	return ''.join(['1' if x else '0' for x in boolean])

# def int2binary(integer):
# 	return '{0:08b}'.format(integer)

# def gen_bins():
# 	bins = [0]*2**numFeatures
# 	return bins

# def update_bins(feature):
# 	bins = gen_bins()
# 	for i in range(len(bins)):
# 		if int2binary(i) == feature:
# 			bins[i] += 1
# 	return bins


