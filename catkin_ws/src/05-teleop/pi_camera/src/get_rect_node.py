#!/usr/bin/env python

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from pi_camera.camera_info import load_camera_info_2
from pi_camera.camera_info import get_camera_info_for_robot
from duckietown_msgs.msg import BoolStamped


#
#img   = cv2.imread('/home/jquack/Desktop/IMG/raw_small.jpg')


#calib = '/home/jquack/duckiefleet/calibrations/camera_intrinsic/trafficlight23.yaml'
#camera_info_msg = load_camera_info_2(calib)



#
# def undistort(img_path):
#     img = cv2.imread(img_path)
#     h,w = img.shape[:2]
#     map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
#     undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
#     cv2.imwrite('/home/jquack/Desktop/IMG/raw2_undistort.jpg', undistorted_img)
#
# undistort('/home/jquack/Desktop/IMG/raw2_small.jpg')

class ImgRect(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.active = True
        self.bridge = CvBridge()

        self.pub_rect = rospy.Publisher("image_rect_new",Image,queue_size=1)
        self.sub_compressed_img = rospy.Subscriber("image_compressed",CompressedImage,self.cbImg,queue_size=1)
        #self.sub_raw_img = rospy.Subscriber("image_raw",Image,self.cbImg,queue_size=1)
        self.robot_name = rospy.get_namespace()
        self.robot_name = self.robot_name[1:-1]

        # this is entireliy unessecary but ued for debugging:
        #self.sub_raw_camera_info = rospy.Subscriber("raw_camera_info", )

        self.camera_info_msg = get_camera_info_for_robot(self.robot_name)
        D=self.camera_info_msg.D
        K=self.camera_info_msg.K

        self.K = np.array([[K[0], K[1], K[2]],[K[3], K[4], K[5]],[K[6], K[7], K[8]]])
        self.D = np.array([[D[0]],[D[1]],[D[2]],[D[3]] ])

        # self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)


    def cbSwitch(self,switch_msg):
        self.active = switch_msg.data

    def cbImg(self,msg):
        if not self.active:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        result_img = cv2.undistort(cv_image, self.K, self.D)
        img_msg = self.bridge.cv2_to_imgmsg(result_img, "bgr8")

        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_rect.publish(img_msg)


# cv2.imwrite('/home/jquack/Desktop/IMG/raw_undistort.jpg', undistorted_img)


if __name__ == '__main__':
    rospy.init_node('get_rect_node',anonymous=False)

    node = ImgRect()
    rospy.spin()
