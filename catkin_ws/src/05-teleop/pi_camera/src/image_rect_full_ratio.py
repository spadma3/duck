#!/usr/bin/env python

#This nodes insert a better calibration to image pipeline for Apriltag detections (full ratio)
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image,CameraInfo
from duckietown_msgs.msg import BoolStamped

import duckietown_utils as dtu
from ground_projection.configuration import get_homography_default, \
    disable_old_homography
from ground_projection.ground_projection_geometry import GroundProjectionGeometry
from ground_projection.ground_projection_interface import estimate_homography, \
     HomographyEstimationResult, save_homography
from pi_camera.camera_info import get_camera_info_for_robot


class ImgRectFullRatio(object):
    """docstring for ImgRectFullRatio"""
    def __init__(self):
        self.node_name = rospy.get_name()
        self.active = True
        self.bridge = CvBridge()

        self.cam_info = None
        self.gpg = None

        self.pub_rect = rospy.Publisher("image_rect",Image,queue_size=1)
        self.pub_cam_info = rospy.Publisher("rect_camera_info",CameraInfo,queue_size=1)
        self.sub_img = rospy.Subscriber("image_raw",Image,self.cbImg,queue_size=1)
        self.sub_cam_info = rospy.Subscriber("camera_info",CameraInfo,self.cbCamInfo,queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)

    def cbSwitch(self,switch_msg):
        self.active = switch_msg.data

    def cbCamInfo(self,caminfo_msg):
        if not self.active:
            return

        self.cam_info = caminfo_msg

        if self.gpg is None:
            #print "Run initialize gpg"
            robot_name = rospy.get_namespace()
            robot_name = robot_name[1:-1]
            disable_old_homography(robot_name)
            homography_dummy = get_homography_default()
            self.gpg = GroundProjectionGeometry(self.cam_info, homography_dummy)

    def cbImg(self,msg):
        if not self.active:
            return
        if self.cam_info is None:
            return
        if self.gpg is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")

        # undistort the image
        # Oriin ratio = 1.65, change to lower ratio to increase the speed of apriltags detection
        new_matrix, result_img = self.gpg.rectify_full(cv_image, ratio=1.65)

        # resulting image has a different size
        # bring back the image to the old size, also crop a little from the outside
        # code from: https://docs.opencv.org/3.4.0/da/d6e/tutorial_py_geometric_transformations.html
        # Preferable interpolation methods are cv2.INTER_AREA for shrinking and cv2.INTER_CUBIC (slow) & cv2.INTER_LINEAR for zooming

        #crop
        # code is from here: https://stackoverflow.com/questions/15589517/how-to-crop-an-image-in-opencv-using-python
        # crop the height from pixel 120 to increase the speed of apriltags detection
        # cut_height = 120
        # result_img = result_img[cut_height:result_img.shape[0], 0:result_img.shape[1]]
        # new_matrix[0, 2] = new_matrix[0, 2]
        # new_matrix[1, 2] = new_matrix[1, 2] - cut_height

        # resize
        # result_img = cv2.resize(result_img,(cv_image.shape[1], cv_image.shape[0]), interpolation = cv2.INTER_AREA)
        # ratio = 1
        # result_img = cv2.resize(result_img,(int(result_img.shape[1]*ratio), int(result_img.shape[0]*ratio)), interpolation = cv2.INTER_AREA)

        img_msg = self.bridge.cv2_to_imgmsg(result_img, "mono8")
        #print "Old Image h,w =", cv_image.shape
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id

        rect_cam_info = self.cam_info
        rect_cam_info.header.stamp = img_msg.header.stamp
        rect_cam_info.header.frame_id = img_msg.header.frame_id
        rect_cam_info.height = result_img.shape[0]
        rect_cam_info.width = result_img.shape[1]

        #add new camera info matrix to the camera info message
        #print "new_matrix", new_matrix
        new_K = tuple(new_matrix.reshape(-1).tolist()[0])
        #print "new_K, ", new_K
        rect_cam_info.K = new_K

        #print "Image h, w = ", rect_cam_info.height, rect_cam_info.width
        self.pub_rect.publish(img_msg)
        self.pub_cam_info.publish(rect_cam_info)


if __name__ == '__main__':
    rospy.init_node('image_rect_full_ratio',anonymous=False)
    node = ImgRectFullRatio()
    rospy.spin()
