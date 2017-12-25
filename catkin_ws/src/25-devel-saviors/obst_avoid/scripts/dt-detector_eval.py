#!/usr/bin/env python

import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
import os
import shutil
from skimage import measure
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify, rgb_from_ros
from duckietown_utils.image_jpg_create import d8_compressed_image_from_cv_image
from obst_avoid.detector import Detector
from obst_avoid.visualizer import Visualizer
from numpy.linalg import inv

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import time
#this script is here to evaulate the perofrmance of our code on whole large datasets
robot_name='arki' #TO BE SET!!!
#----------------SPECIFY FILES TO READ IN ---------------------
#ASSUMES: pictures stored in folder structure ..../3/5_pics/"here_are_all_pics"
start_file=6 #represent 1_pics e.g. start_file=end_file=1 only evaluates first folder!!!
end_file=6
name_new_folder="edited"
general_path = '/home/niggi/savior_bags/19.12.bags/1/'
dir_path = general_path+str(start_file)+"_pics"+"/"+name_new_folder
im_path = general_path+str(start_file)+"_pics"
#print dir_path




rospy.init_node('obstacle_detection_node',disable_signals=True)
detector = Detector(robot_name=robot_name)
intrinsics = load_camera_intrinsics(robot_name)
visualizer = Visualizer(robot_name=robot_name)
H = load_homography(robot_name)

obst_list = PoseArray()
marker_list = MarkerArray()


#CREATE NEW DIRECTORY
if os.path.exists(dir_path):
    shutil.rmtree(dir_path)
os.makedirs(dir_path)
#cv2.imwrite(dir+ '/' + str(i) + '.jpg',im1)

nummer=1

while(True):
    filename = im_path+ '/' + str(nummer) + '.jpg'
    im1 = cv2.imread(filename) #reads BGR
    if (im1 is None):
        #zum naechsten Ordner gehen!!!
        if (start_file>end_file):
            break
        else:
            start_file+=1
            if (start_file>end_file):
                break
            nummer=1
            dir_path = general_path+str(start_file)+"_pics"+"/"+name_new_folder
            im_path = general_path+str(start_file)+"_pics"
            #CREATE NEW DIRECTORY
            if os.path.exists(dir_path):
                shutil.rmtree(dir_path)
            os.makedirs(dir_path)
    
    
    else: #START MODIFYING THE IMAGE!!!
        #-------------HERE GOES THE REAL CODE-----------------------------------------------------------
        #-----------------------------------------------------------------------------------------------

        obst_list = detector.process_image(rectify(im1[:,:,::-1],intrinsics))
        obst_image = CompressedImage()
        obst_image.format = "jpeg"
        obst_image.data = visualizer.visualize_image(rectify(im1[:,:,::-1],intrinsics),obst_list)
        #here i want to display cropped image
        image=rgb_from_ros(obst_image.data)
        #THIS part only to visualize the cropped version -> somehow a little inefficient but keeps
        #the visualizer.py modular!!!
        #plt.imshow(image[detector.crop:,:,:]);plt.show() #the cropped image
        #plt.imshow(image);plt.show()                     #normal sized image
        #SAVE THE IMAGE
        conv = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(dir_path+ '/' + str(nummer) + '.jpg', conv)
        nummer+=1
        

print "FERTIG"
os.system("rosnode kill obstacle_detection_node")