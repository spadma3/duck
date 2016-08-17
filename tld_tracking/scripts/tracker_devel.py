#!/usr/bin/env python
import cv2
import argparse
import numpy as np
from time import time

cv2.namedWindow("image", cv2.WINDOW_NORMAL)
video = cv2.VideoCapture("/home/ubuntu/Original_Images_Bag/run_video/extract_run3/output3.mpg")
# print video.isOpened()



flag, frame = video.read()
pos_frame = video.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
print flag
print pos_frame
cv2.imshow("image", frame)
cv2.waitKey(3000)

flag, frame = video.read()
pos_frame = video.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
pos_frame2 = video.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 30)
print pos_frame
cv2.imshow("image", frame)
cv2.waitKey(3000)

# cv2.rectangle(rec, )

# flag2, frame2 = video.read()
# print flag2
# cv2.imshow("image", frame2)
# pos_frame = video.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
# print pos_frame
# cv2.waitKey(3000)




video.release()
cv2.destroyAllWindows()

# import cv2

# cap = cv2.VideoCapture("/home/ubuntu/Original_Images_Bag/run_video/extract_run3/output3.mpg")
# while not cap.isOpened():
#     cap = cv2.VideoCapture("/home/ubuntu/Original_Images_Bag/run_video/extract_run3/output3.mpg")
#     cv2.waitKey(1000)
#     print "Wait for the header"

# pos_frame = cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
# while True:
#     flag, frame = cap.read()
#     if flag:
#         # The frame is ready and already captured
#         cv2.imshow('video', frame)
#         pos_frame = cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)
#         print str(pos_frame)+" frames"
#     else:
#         # The next frame is not ready, so we try to read it again
#         cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, pos_frame-1)
#         print "frame is not ready"
#         # It is better to wait for a while for the next frame to be ready
#         cv2.waitKey(1000)

#     if cv2.waitKey(10) == 27:
#         break
#     if cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES) == cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT):
#         # If the number of captured frames is equal to the total number of frames,
#         # we stop
#         break


# ref_points = []
# bounding_boxes = []


# def mouse_draw(event, x, y, flags, param):
# 	global ref_points, bounding_boxes
# 	# print "In mouse callback"
# 	if event == cv2.EVENT_LBUTTONDOWN:
# 		ref_points = [(x,y)]
# 		print ref_points
# 		print "The key is pressed, value %d, %d" %(ref_points[0][0], ref_points[0][1])
# 		mouse_drag = True

# 	# elif event == cv2.EVENT_MOUSEMOVE:
# 	# 	print "Mouse is moving"
# 	# 	# print ref_points
# 	# 	cv2.rectangle(image, ref_points[0], (x,y), (0,255,0),2)
# 	# 	cv2.waitKey(10)

# 	elif event == cv2.EVENT_LBUTTONUP:
# 		cv2.rectangle(image, ref_points[0], (x,y), (0,255,0),2)
# 		cv2.waitKey(10)
# 		# ref_points.append((x,y))
# 		# print ref_points
# 		# cv2.rectangle(image, ref_points[0], ref_points[1], (0,255,0),2)
# 		# bounding_boxes.append([ref_points[0], ref_points[1]])
# 		# print bounding_boxes
# 		# print "The key is released, value %d, %d" %(ref_points[1][0], ref_points[1][1])
# 	cv2.imshow("image", image)
# 	cv2.waitKey(30)


# argument_object = argparse.ArgumentParser()
# argument_object.add_argument("-i", "--image", required=True, help="Path to the image")
# arguments = vars(argument_object.parse_args())

# image = cv2.imread(arguments["image"])
# clone_image = image.copy()
# cv2.namedWindow("image")
# cv2.setMouseCallback("image", mouse_draw)

# while True:
# 	key = cv2.waitKey(1) & 0xFF

# 	if key == ord("r"):
# 		image = clone_image.copy()

# 	if key == ord("c"):
# 		break
# cv2.destroyAllWindows()


# def gen_point_cloud(box, density, margin):
# 	pts = [] 		# [(x1,y1),(x2,y2)...]
# 	img_track = [(box[0][0]+margin, box[0][1]+margin),(box[1][0]-margin,box[1][0]-margin)]
# 	numY = ((img_track[1][1] - img_track[0][1])/density) + 1
# 	numX = (img_track[1][0] - img_track[0][0])/density + 1
# 	for i in range(numX):
# 		for j in range(numY):
# 			pts_x = box[0][0] + i*density
# 			pts_y = box[0][1] + j*density
# 			pts.append((pts_x,pts_y))
# 	return pts, len(pts)


# print gen_point_cloud([(30,30),(41,41)],2,0)

# def gen_point_cloud(box, numY, numX, margin):
# 	pts2 = [] 		# [(x1,y1),(x2,y2)...]
# 	img_track = [(box[0][0]+margin, box[0][1]+margin),(box[1][0]-margin,box[1][0]-margin)]
# 	# spaceY = (img_track[1][1] - img_track[0][1])/(numY - 1)
# 	# spaceX = (img_track[1][0] - img_track[0][0])/(numX - 1)
# 	try:
# 		spaceY = (img_track[1][1] - img_track[0][1])/(numY - 1)
# 		spaceX = (img_track[1][0] - img_track[0][0])/(numX - 1)
# 	except ZeroDivisionError:
# 		print "Error...Please draw an appropriate box"
# 	for i in range(numX):
# 		for j in range(numY):
# 			pts_x = box[0][0] + i*spaceX
# 			pts_y = box[0][1] + j*spaceY
# 			pts2.append((pts_x,pts_y))
# 	return pts2, len(pts2)
# print gen_point_cloud([(30,30),(41,41)], 6, 6, 0)

# def cal_target_points(img1, img2, pts0):
# 	valid_target_pts = [] # initialize the target points with equal length to source
# 	valid_start_pts = []
# 	start_pts = np.asarray(pts0, dtype="float32")
# 	lk_params = dict(winSize=(15,15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | \
# 		cv2.TERM_CRITERIA_COUNT,10,0.03),flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
# 	matching_param = dict(winSize_match=4, method=cv2.cv.CV_TM_CCOEFF_NORMED)

# 	target_pts, status_forward,_ = cv2.calcOpticalFlowPyrLK(img1,img2,start_pts,**lk_params) #[(x1,y1),(x2,y2)...]
# 	back_pts, status_backward,_ = cv2.calcOpticalFlowPyrLK(img2,img1,target_pts,**lk_params)
	
# 	dist_all = euclidean_distance(start_pts, target_pts)
# 	valid_corr = patch_matching(img1,img2,start_pts,target_pts,**matching_param)   # -1 for untracked points, higher value better corr
# 	valid_dist = []   # -1 for untracked points

# 	status = status_forward & status_backward
# 	match_patches = np.zeros(len)
# 	for [i] in np.argwhere(status):
# 		valid_target_pts.append(target_pts[i])
# 		valid_start_pts.append(start_pts[i])
# 		valid_dist.append(dist_all[i])

# 	test = len(valid_start_pts) == len(valid_target_pts) == len(valid_dist) == len(valid_corr)
# 	print test, "New points and their distances"

# 	return valid_target_pts 

# # Find the match between start and target patches around the points specified by winSize
# def patch_matching(img1,img2,valid_start_pts,valid_target_pts,status,winSize,method):
# 	match_patches = []
# 	for [i] in np.argwhere(status):
# 		patch_start = cv2.getRectSubPix(img1,(winSize,winSize),start_pts[i])
# 		patch_target = cv2.getRectSubPix(img2,(winSize,winSize),target_pts[i])
# 		match_patches.append(cv2.matchTemplate(patch_start,patch_target,method))
# 	return match_patches

# def euclidean_distance(start_pts,target_pts):
# 	dist = ((target_pts[:,0]-start_pts[:,0])**2 + (target_pts[:,1]-start_pts[:,1])**2)**0.5
# 	return dist 


# def filter_valid_points(valid_corr, valid_dist, valid_target_pts, valid_start_pts):
# 	good_target_points = []
# 	good_start_points = []
# 	medDist = median(valid_dist)
# 	medCorr = median(valid_corr)
# 	for i in range(len(valid_dist)):
# 		if valid_dist[i] <= medDist & valid_corr[i] >= medCorr:
# 			good_target_points.append(valid_target_pts[i])
# 			good_start_points.append(valid_target_pts[i])
# 	num_target_pts = len(good_target_points)
# 	return good_points

# def median(data):
# 	new_data = copy(data)
# 	new_data.sort()
# 	if len(new_data) <1:
# 		print "No Data point to calculate median"
# 		return None
# 	if len(new_data)%2 == 1:
# 		return new_data[len(new_data)/2]
# 	else:
# 		return float(sum(new_data[(len(new_data)/2)-1:(len(new_data)/2)+1]))

# def predict_bounding_box(start_box,good_start_points, good_target_points, num_target_pts):
# 	width_start = start_box[1][0] - start_box[0][0]
# 	height_start = start_box[1][1] - start_box[0][1]
# 	diff_x = []
# 	diff_y = []
# 	for i in range(num_target_pts):
# 		diff_x.append(good_target_points[i][0] - good_start_points[i][0])
# 		diff_y.append(good_target_points[i][1] - good_start_points[i][1])
# 	dx = median(diff_x)
# 	dy = median(diff_y)
# 	diff_y = diff_x = 0

# 	scale_factor = []
# 	for i in range(num_target_pts):
# 		for j in range(i+1, num_target_pts):
# 			start_img = ((good_start_points[i][0]-good_start_points[j][0])**2 \
# 			(good_start_points[i][1] - good_start_points[j][1])**2)**0.5
# 			target_img = ((good_target_points[i][0]-good_target_points[j][0])**2 \
# 			(good_target_points[i][1] - good_target_points[j][1])**2)**0.5
# 			frac_shift.append(float(target_img)/start_img)
# 	scale = median(scale_factor)
	
# 	if scale == 0:
# 		scale = 1
# 	scale_x = ((scale -1)/2)*width_start
# 	scale_y = ((scale-1)/2)*height_start

# 	x1_new = start_box[0][0] + dx - scale_x
# 	x2_new = start_box[1][0] + dx + scale_x
# 	y1_new = start_box[0][1] + dy - scale_y
# 	y2_new = start_box[1][1] + dy + scale_y

# 	target_box = [(x1_new,y1_new), (x2_new, y2_new)]
# 	return target_box
