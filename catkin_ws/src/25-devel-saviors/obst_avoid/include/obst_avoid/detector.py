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
	self.lower_yellow = np.array([20,75,100])
	self.upper_yellow = np.array([40,255,255])
	

	# initialize second publisher, later i think we should put this in the "front" file
	# currently we publish the "bottom" center of the obstacle!!!
	#self.pub_topic2 = '/{}/obst_coordinates'.format(robot_name)
    	#self.publisher2 = rospy.Publisher(self.pub_topic2, Point, queue_size=1)
	
    def process_image(self, image):
    	obst_list = PoseArray()
	# FILTER CROPPED IMAGE
	# Convert BGR to HSV
	hsv = cv2.cvtColor(image[self.crop:,:,:], cv2.COLOR_RGB2HSV)
	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
	#die Randpixel auf null setzen sonst unten probleme mit den boundaries
	mask[0,:]=0
	mask[np.size(mask,0)-1,:]=0
	mask[:,0]=0
	mask[:,np.size(mask,1)-1]=0
		
	if(np.sum(mask!=0)!=0): #there were segment detected then
		#SEGMENT IMAGE
		segmented_image=self.segment_img(mask)

		#apply filter on elements-> only obstacles remain and mark them in original picture
		#in the future: might be separated in 2 steps 1)extract objects 2)visualisation		
		obst_list = self.object_filter(segmented_image)

	return obst_list


    def segment_img(self, image):
		#returns segmented image on Grayscale where all interconnected pixels have same number
		return measure.label(image)



    def object_filter(self,segmented_img):
	#for future: filter has to become adaptive to depth
	obst_list = PoseArray()
   	obst_list.header.frame_id=self.robot_name
	i=np.max(segmented_img)
	for k in range(1,i+1): #iterate through all segmented numbers
		#first only keep large elements then eval their shape
		if (np.sum((segmented_img == k))<100): #skip all those who were merged away or have not enough pixels tiefenabh???
		    segmented_img[(segmented_img == k)]=0
		else:
		    
		    B=np.copy(segmented_img)
		    B[(B != k)]=0
		    C=np.nonzero(B)
		    ITER=np.reshape(C, (2,-1))
		    #print C 
		    top=np.min(C[0])
		    bottom=np.max(C[0])
		    left=np.min(C[1])
		    right=np.max(C[1])
		    height=bottom-top #indices are counted from top to down
		    total_width=right-left
		    
		    # Now finidng the width on the same height
		    height_left=np.max(ITER[0,(C[1]==left)]) 
		    width_height_left = np.max(ITER[1,(C[0]==height_left)])
		    #WIDTH AT HEIGHT OF LEFT POSITION
		    width_left= width_height_left-left

		    height_right=np.max(ITER[0,(C[1]==right)]) #ACHTUNG:kann mehrere WERTE HABEN
		    width_height_right = np.min(ITER[1,(C[0]==height_right)])
		    #WIDTH AT HEIGHT OF RIGHT POSITION
		    width_right= right-width_height_right

		    #print width_right
		    #print width_left
		    #print width_right
		    #print height_left
		    #print height_right

		    #print "NEW OBJECT:"
		    #print bottom
		    #print width_right
		    #print 1.0*width_right/bottom

		    #WIDTH AT TOP
		    #MUSS NOCH ABFRAGE HIN DAMIT MAN NICHT OUT OF BOUNDS LAEUFT
		    #width_top = np.max(ITER[1,(C[0]==top+int(0.5*height))])-np.min(ITER[1,(C[0]==top+int(0.5*height))])

		    #if (abs(height_left-height_right)>10): #FILTER 1
		    #    final[(final == k)]=0
		    #if (abs(width_top-width_right)<20): #FILTER 2
		    #    final[(final == k)]=0

		    if (1.0*width_right/bottom<0.25): #FILTER 3
		        segmented_img[(segmented_img == k)]=0

		    else:
		        #UEBERGABE?
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
			obst_object.orientation.x = left
			obst_object.orientation.y = top
			obst_object.orientation.z = right
			obst_object.orientation.w = bottom

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
