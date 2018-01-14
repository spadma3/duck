#!/usr/bin/env python
# -*- coding: utf-8 -*-2

import rospy
import numpy as np
import roslib
from image_geometry import PinholeCameraModel
import sys
import cv2
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
  	#self.node_name = rospy.get_name()
    self.node_name="image_proc_proportional_node"
    #self.robot_name = "schumi" 
    self.robot_name = rospy.get_param("~veh")

    self.pcm = PinholeCameraModel()

    self.active = True
    self.bridge=CvBridge()

    self.sub_raw = rospy.Subscriber("/"+self.robot_name+"/camera_node/image/raw", Image, self.callback)
    self.sub_raw = rospy.Subscriber("/"+self.robot_name+"/camera_node/raw_camera_info", CameraInfo, self.CIcallback)
    self.pub_rect  = rospy.Publisher("~image_rect", Image, queue_size=1) #, latch=True

    self.stamp = rospy.Time.now()

    #camera_info_topic = "/"+self.robot_name+"/camera_node/camera_info"
    camera_info_topic = "/" + self.robot_name + "/camera_node/raw_camera_info"
    rospy.loginfo("camera info topic is " + camera_info_topic)
    rospy.loginfo("waiting for camera info")
    camera_info = rospy.wait_for_message(camera_info_topic,CameraInfo)
    rospy.loginfo("camera info received")
    self.initialize_pinhole_camera_model(camera_info)


# wait until we have recieved the camera info message through ROS and then initialize
  def initialize_pinhole_camera_model(self,camera_info):
    #self.ci=camera_info
    self.pcm.fromCameraInfo(camera_info)
    print("pinhole camera model initialized")


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.rectify_full(cv_image)

    try:
      img_rect = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
      #img_rect.header.stamp = rospy.Time.now()
      img_rect.header.stamp = self.stamp
      self.pub_rect.publish(img_rect)
    except CvBridgeError as e:
      print(e)

  def CIcallback(self,data):
    self.stamp = data.header.stamp

  def rectify_full(self, cv_image_raw, interpolation=cv2.INTER_NEAREST):

    W = self.pcm.width
    H = self.pcm.height

    print ("W: ", W)
    print ("H: ", H)

    mapx = np.ndarray(shape=(H, W, 1), dtype='int')
    mapy = np.ndarray(shape=(H, W, 1), dtype='int')
    #print('K: %s' % self.pcm.K)
    #print('P: %s' % self.pcm.P)

#        alpha = 1
#        new_camera_matrix, validPixROI = cv2.getOptimalNewCameraMatrix(self.pcm.K, self.pcm.D, (H, W), alpha)
#        print('validPixROI: %s' % str(validPixROI))
#        print('new_camera_matrix: %s' % new_camera_matrix)

    # Use the same camera matrix
    new_camera_matrix = self.pcm.K
    mapx, mapy = cv2.initUndistortRectifyMap(self.pcm.K, self.pcm.D, self.pcm.R,
                                             new_camera_matrix, (W, H),
                                             cv2.CV_32FC1, mapx, mapy)
    cv_image_rectified = np.empty_like(cv_image_raw)
    res = cv2.remap(cv_image_raw, mapx, mapy, interpolation,
                    cv_image_rectified)
    return res


if __name__ == '__main__':
    rospy.init_node('image_proc_proportional_node', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
