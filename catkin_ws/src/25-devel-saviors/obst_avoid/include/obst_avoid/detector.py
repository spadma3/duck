#!/bin/bash/env python

import argparse
import cv2
import numpy as np
from numpy.linalg import inv
from os.path import basename, expanduser, isfile, join, splitext
import socket
from matplotlib import pyplot as plt
import time
from skimage import measure

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify

class Detector():
    '''class for detecting obstacles'''
    def __init__(self, robot_name='',crop_rate=150):
        # Robot name
    	self.robot_name = robot_name

        # Load camera calibration parameters
	self.intrinsics = load_camera_intrinsics(robot_name)
	self.H = load_homography(self.robot_name)

	#define where to cut the image, color range,...
	self.crop=crop_rate #default value=150 see above!!!
	self.lower_yellow = np.array([20,100,150])
	self.upper_yellow = np.array([35,255,255])
	self.img_width = 0 #to be set in init_inv_homography
	self.img_height = 0 #to be set in init_inv_homography
	self.M = self.init_inv_homography()
	self.inv_M = inv(self.M)


    def init_inv_homography(self):
    	x0=0
	x1=640
	y0=50
	y1=350
	pts1 = np.float32([[x0,y0],[x0,y1],[x1,y1],[x1,y0]])
	pts1_h = np.float32([[x0,y0+self.crop,1],[x0,y1+self.crop,1],[x1,y1+self.crop,1],[x1,y0+self.crop,1]])
	#add the crop offset to being able to calc real world coordinates correctly!!!
	pts2_h = np.dot(self.H,np.transpose(pts1_h))

	pts2 = np.float32((pts2_h[0:2,:]/pts2_h[2,:]*1000))
	maximum_height = np.max([pts2[0,:]])
	maximum_left = np.max([pts2[1,:]])
	#print pts2
	#determine points number 2!!!
	pts2 = np.flipud(np.float32((np.float32([[maximum_height],[maximum_left]])-pts2)))
	#flipud only cause world frame is flipped,..

	self.img_width = int(np.max(pts2[0]))
	self.img_height = int(np.max(pts2[1]))
	return cv2.getPerspectiveTransform(pts1,np.transpose(pts2))

		
    def process_image(self, image):
    	obst_list = PoseArray()
	# FILTER CROPPED IMAGE
	# Convert BGR to HSV
	hsv = cv2.cvtColor(image[self.crop:,:,:], cv2.COLOR_RGB2HSV)
	# Threshold the HSV image to get only yellow colors
	im_test = cv2.warpPerspective(hsv,self.M,(self.img_width,self.img_height)) 

	mask = cv2.inRange(im_test, self.lower_yellow, self.upper_yellow)

		
	if(np.sum(mask!=0)!=0): #there were segment detected then
		#SEGMENT IMAGE
		segmented_image=self.segment_img(mask)
		props=measure.regionprops(segmented_image)
		no_elements = np.max(segmented_image)
		#apply filter on elements-> only obstacles remain and mark them in original picture
		#in the future: might be separated in 2 steps 1)extract objects 2)visualisation		
		obst_list = self.object_filter(props,no_elements)

	return obst_list


    def segment_img(self, image):
		#returns segmented image on Grayscale where all interconnected pixels have same number
		return measure.label(image)



    def object_filter(self,props,no_elements):
	#for future: filter has to become adaptive to depth
	obst_list = PoseArray()
   	obst_list.header.frame_id=self.robot_name
	for k in range(1,no_elements+1): #iterate through all segmented numbers
		#first only keep large elements then eval their shape
		if (props[k-1]['area']>self.img_width): #skip all those who were merged away or have not enough pixels tiefenabh???
		    	top=props[k-1]['bbox'][0]
			bottom=props[k-1]['bbox'][2]
		      	left=props[k-1]['bbox'][1]
		        right=props[k-1]['bbox'][3]
		        total_width = right-left

		        obst_object = Pose()
			point_calc=np.zeros((3,1),dtype=np.float)
			point_calc=self.pixel2ground([[left+0.5*total_width],[bottom+self.crop]])
			#take care cause image was cropped,..
			obst_object.position.x = point_calc[0] #obstacle coord x
			obst_object.position.y = point_calc[1] #obstacle coord y
			#calculate radius:
			point_calc2=np.zeros((3,1),dtype=np.float)
			point_calc2=self.pixel2ground([[left],[bottom+self.crop]])
			obst_object.position.z = point_calc2[1]-point_calc[1] #this is the radius!

			#fill in the pixel boundaries
			obst_object.orientation.x = top
			obst_object.orientation.y = bottom
			obst_object.orientation.z = left
			obst_object.orientation.w = right

			obst_list.poses.append(obst_object)
		 
			#explanation: those parameters published here are seen from the !center of the axle! in direction
			#of drive with x pointing in direction and y to the left of direction of drive in [m]		        
			#cv2.rectangle(orig_img,(np.min(C[1]),np.min(C[0])),(np.max(C[1]),np.max(C[0])),(0,255,0),3)

	    #eig box np.min breite und hoehe!! if they passed the test!!!!
	    #print abc
	    
	return obst_list

    def pixel2ground(self,pixel):
    	#taking pixel coordinates with (column,row) and returning real world coordinates!!!
	point_calc=np.zeros((3,1),dtype=np.float)
	point_calc= np.dot(self.H,[pixel[0],pixel[1],[1]]) #calculating realWorldcoords
	point_calc=[(point_calc[0])/point_calc[2],(point_calc[1])/point_calc[2],1]
	return point_calc
