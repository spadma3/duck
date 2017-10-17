import cv2
import numpy as np
from os.path import basename, expanduser, isfile, join, splitext

from duckietown_utils import logger, yaml_load, get_duckiefleet_root

def load_map(map_filename):
    if not isfile(map_filename):
        print('Map does not exist')
        exit(1)
    map_name = splitext(basename(map_file))[0]
	with open(map_file) as f:
		contents = f.read()
		data = yaml_load(contents)
	logger.info('Loaded {} map data'.format(map_name))
	return data

def load_homography(robot_name):
	path = '/calibrations/camera_'
	filename = get_duckiefleet_root() + path + 'extrinsic/' + robot_name + '.yaml'
	if not isfile(filename):
		print('Extrinsic calibration for {} does not exist.'.format(robot_name))
		exit(2)
	with open(filename) as f:
		contents = f.read()
		data = yaml_load(contents)
	logger.info('Loaded homography for {}'.format(robot_name))
	return np.array(data['homography']).reshape(3,3)

def load_camera_intrincs(robot_name):
	path = '/calibrations/camera_'
	filename = get_duckiefleet_root() + path + 'intrinsic/' + robot_name + '.yaml'
	if not isfile(filename):
		print('Intrinsic calibration for {} does not exist.'.format(robot_name))
		exit(3)
	with open(filename) as f:
		contents = f.read()
		data = yaml_load(contents)
	intrinsics = {}
	intrinsics['K'] = np.array(data['camera_matrix']['data']).reshape(3, 3)
	intrinsics['D'] = np.array(data['distortion_coefficients']['data']).reshape(1, 5)
	intrinsics['R'] = np.array(data['rectification_matrix']['data']).reshape(3, 3)
	intrinsics['P'] = np.array(data['projection_matrix']['data']).reshape((3,4))
	intrinsics['distortion_model'] = data['distortion_model']
	logger.info('Loaded camera intrinsics for {}'.format(robot_name))
	return intrinsics

def rectify(image, intrinsics):
	'''Undistort image'''
	height, width, channels = image.shape
	rectified_image = np.zeros(np.shape(image))
	mapx = np.ndarray(shape=(height, width, 1), dtype='float32')
	mapy = np.ndarray(shape=(height, width, 1), dtype='float32')
	mapx, mapy = cv2.initUndistortRectifyMap(intrinsics['K'], intrinsics['D'], intrinsics['R'], intrinsics['P'], (width, height), cv2.CV_32FC1, mapx, mapy)
	return cv2.remap(image, mapx, mapy, cv2.INTER_CUBIC, rectified_image)
