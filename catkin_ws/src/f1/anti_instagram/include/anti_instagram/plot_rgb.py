#!/usr/bin/env python
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
#from sensor_msgs.msg import CompressedImage,Image

import random

def plot(r,g,b,centers=None):
	# cv2.imshow('image',img)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	# numSamples = 20
	# r_rand,g_rand,b_rand = sample(img,numSamples)	

	# Plot rgb as xyz
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(r, g, b)
	if centers is not None:
		ax.scatter(centers[0][0],centers[0][1],centers[0][2],s=200,c='red')
		ax.scatter(centers[1][0],centers[1][1],centers[1][2],s=200,c='green')
		ax.scatter(centers[2][0],centers[2][1],centers[2][2],s=200,c='brown')
	ax.set_xlabel('R')
	ax.set_ylabel('G')
	ax.set_zlabel('B')
	plt.show()

def split(img):
	r = [img[i][0] for i in range(len(img))]
	g = [img[i][1] for i in range(len(img))]
	b = [img[i][2] for i in range(len(img))]

	return r,g,b

def splitAndSample(img,numSamples):
	r,g,b = cv2.split(img)
	img_height = len(r)
	img_width = len(r[0])
	# top_index = int(img_height*2/3.0)
	top_index = 0

	# Randomly sample numSampes^2 pixels from the image (uniformly distributed in x,y)
	iList = [random.randint(top_index,len(r)-1) for i in range(numSamples)]
	jList = [random.randint(0,img_width) for i in range(numSamples)]
	r_rand = [r[i][j] for i in iList for j in jList]
	b_rand = [b[i][j] for i in iList for j in jList]
	g_rand = [g[i][j] for i in iList for j in jList]
	return r_rand,g_rand,b_rand

def all(img,numSamples,c=None):
	numSamples = 20
	# r,g,b = splitAndSample(img,numSamples)
	# plot(r,g,b,centers=c)
	r,g,b = split(img)
	plot(r,g,b,centers=c)


if __name__=='__main__':
	image_dir = '/home/mfe/'
	image_name = 'test2.jpg'

	# Open image and split into 3 channels
	img = cv2.imread( image_dir+image_name )
	
	all(img,20)


# #for topic, msg, t in bag.read_messages(topics=['/ferrari/camera_node/img_low/compressed']):
# for topic, msg, t in bag.read_messages(topics=['/rosberrypi_cam/image_raw']):

# 	np_arr = np.fromstring(msg.data, np.uint8)
# 	cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
# 	#adj_color = cv2.applyColorMap(cv_image, cv2.COLORMAP_AUTUMN)
# 	height = len(np_arr)
# 	horizon_height = int(height/2.0)
# 	img_slab = np_arr[horizon_height:height]
# 	# b,g,r = cv2.split(img)       # get b,g,r
# 	# rgb_img = cv2.merge([r[n:n+w],g[n:n+w],b[n:n+w]])     # switch it to rgb
# 	#img = Image.fromarray( np.asarray( np.clip(img,0,255), dtype="uint8"), "RGB" )
# 	img = Image.fromarray(img_slab)
# 	img.save( '/home/mfe/test2.png' )
# 	break
# bag.close()

# bag = rosbag.Bag('/home/mfe/Downloads/mercedes_dark2_2016-01-03-21-59-30_4.bag')