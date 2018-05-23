#!/usr/bin/env python

#This nodes insert a better calibration to image pipeline for Apriltag detections (full ratio)
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
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

        self.pub_rect = rospy.Publisher("image_rect",Image,queue_size=1)
        self.sub_compressed_img = rospy.Subscriber("image_raw",Image,self.cbImg,queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)


    def cbSwitch(self,switch_msg):
        self.active = switch_msg.data

    def cbImg(self,msg):
        if not self.active:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        robot_name = rospy.get_namespace()
        robot_name = robot_name[1:-1]
        disable_old_homography(robot_name)

        camera_info = get_camera_info_for_robot(robot_name)
        homography_dummy = get_homography_default()
        gpg = GroundProjectionGeometry(camera_info, homography_dummy)

        new_matrix, result_img = gpg.rectify_full(cv_image, ratio=1.65)

        img_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")

        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_rect.publish(img_msg)


if __name__ == '__main__': 
    rospy.init_node('image_rect_full_ratio',anonymous=False)
    node = ImgRectFullRatio()
    rospy.spin()