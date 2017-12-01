#!/bin/bash/env python

import argparse
import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

from duckietown_utils import d8_compressed_image_from_cv_image, logger, rgb_from_ros, yaml_load, get_duckiefleet_root
from duckietown_utils import get_base_name, load_camera_intrinsics, load_homography, load_map, rectify
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify



class Visualizer():
    '''class for visualizing detected obstacles'''
    def __init__(self, robot_name='',crop_rate=150):
        # Robot name
    	self.robot_name = robot_name

	#define where to cut the image, color range,...
	self.crop=crop_rate #default value=150 see above!!!

	
    def visualize_marker(self, obst_list):
    	marker_list=MarkerArray()
	
	marker = Marker()
	marker.type = marker.CYLINDER
	marker.header.frame_id=self.robot_name
	marker.frame_locked=False
	marker.scale.z = 1.0
	marker.color.a = 1.0
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker.pose.orientation.w = 1.0
	marker.lifetime = rospy.Time(1.0) 
	#each marker if not replaced earlier will stay for 1 second

   	size = obst_list.poses.__len__()
   	for i in range(0,size):


   		marker.id = i
	   	marker.scale.x = obst_list.poses[i].position.z
   		marker.scale.y = obst_list.poses[i].position.z
	   	marker.pose.position.x = obst_list.poses[i].position.x
	   	marker.pose.position.y = obst_list.poses[i].position.y
	   	marker.pose.position.z = 0 
	   	marker_list.markers.append(marker)

    	print marker_list.markers.__len__()
    	return marker_list

    def visualize_image(self, image,obst_list):

    	orig_img=image[self.crop:,:,:]
    	size = obst_list.poses.__len__()
   	for i in range(0,size):

   		cv2.rectangle(orig_img,(obst_list.poses[i].orientation.x,obst_list.poses[i].orientation.y),(obst_list.poses[i].orientation.z,obst_list.poses[i].orientation.w),(0,255,0),3)

	    #eig box np.min breite und hoehe!! if they passed the test!!!!
	    #print abc
	    
	return d8_compressed_image_from_cv_image(orig_img[:,:,::-1])







    def ground2pixel(self, point):
        '''Transforms point in ground coordinates to point in image
        coordinates using the inverse homography'''
	point_calc=np.zeros((3,1),dtype=np.float)
	point_calc= np.dot(inv(self.H),[[point[0]],[point[1]],[1]])

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