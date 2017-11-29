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

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify

class Detector():
    '''class for detecting obstacles'''
    def __init__(self, robot_name=''):
        # Robot name
    	self.robot_name = robot_name

        # Load camera calibration parameters
	self.intrinsics = load_camera_intrinsics(robot_name)
	self.H = inv(load_homography(self.robot_name))	
	
    def process_image(self, image):

	# CROP IMAGE, image is BGR
 	im1_cropped = image[130:,:,:]

        # FILTER IMAGE
	# Convert BGR to HSV
	hsv = cv2.cvtColor(im1_cropped, cv2.COLOR_RGB2HSV)
	lower_yellow = np.array([20,75,100])
	upper_yellow = np.array([40,255,255])
	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
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
		image = self.object_filter(segmented_image,im1_cropped)
	else:
		image = im1_cropped

	# CHANGE BGR BACK TO RGB
	jpg_data = image[:,:,::-1]
	#before sending: REDEFINE DATA 
    	return d8_compressed_image_from_cv_image(jpg_data)


    def segment_img(self, image):
		#returns segmented image on Grayscale where all interconnected pixels have same number
		return measure.label(image)



    def object_filter(self,segmented_img,orig_img):
	#for future: filter has to become adaptive to depth
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
		        #obst_arr[0,entry]=int(np.max(C[0])-0.5*width)
		        #obst_arr[1,entry]=int(np.max(C[1])-0.5*height)
		        #obst_arr[2,entry]=int(0.5*width)
		        #obst_arr[3,entry]=int(0.5*height)
		        #entry+=1
		        cv2.rectangle(orig_img,(np.min(C[1]),np.min(C[0])),(np.max(C[1]),np.max(C[0])),(0,255,0),3)

	    #eig box np.min breite und hoehe!! if they passed the test!!!!
	    #print abc
	    
	return orig_img







    def ground2pixel(self, point):
        '''Transforms point in ground coordinates to point in image
        coordinates using the inverse homography'''
	point_calc=np.zeros((3,1),dtype=np.float)
	point_calc= np.dot(self.H,[[point[0]],[point[1]],[1]])

	pixel_int=[int((point_calc[0])/point_calc[2]),int((point_calc[1])/point_calc[2])]
	#print pixel_float
        return pixel_int

    def just2pixel(self, point):
        '''Draw Lines around picture'''
        return [point[0]*640,point[1]*480]


    def render_segments(self, image):
        for segment in self.map_data["segments"]:	
            pt_x = []
            pt_y = []
            for point in segment["points"]:
                frame, ground_point = self.map_data["points"][point]
                pixel = []
                if frame == 'axle':
                    pixel = self.ground2pixel(ground_point)
                elif frame == 'camera':
                    pixel = ground_point
                elif frame == 'image01':
                    pixel = self.just2pixel(ground_point)
                else:
                    logger.info('Unkown reference frame. Using "axle" frame')
                    pixel = self.ground2pixel(ground_point)
                pt_x.append(pixel[0])
                pt_y.append(pixel[1])
            color = segment["color"]
            image = self.draw_segment(image, pt_x, pt_y, color)
        return image

    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red' : ['rgb', [1, 0, 0]],
            'green' : ['rgb', [0, 1, 0]],
            'blue' : ['rgb', [0, 0, 1]],
            'yellow' : ['rgb', [1, 1, 0]],
            'magenta' : ['rgb', [1, 0 ,1]],
            'cyan' : ['rgb', [0, 1, 1]],
            'white' : ['rgb', [1, 1, 1]],
            'black' : ['rgb', [0, 0, 0]]}
        color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]),(pt_x[1], pt_y[1]),(b * 255, g* 255, r * 255), 5)
        return image
