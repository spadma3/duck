#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from picamera import PiCamera
from picamera.array import bytes_to_yuv
import time
import signal
import sys
import rospkg
import os.path
import yaml

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        print "initializing"

        self.bridge = CvBridge()

        self.framerate = self.setupParam("~framerate",15.0)
        self.res_w = self.setupParam("~res_w",640)
        self.res_h = self.setupParam("~res_h",480)

        # For intrinsic calibration
        rospack = rospkg.RosPack()
        self.config = self.setupParam("~config","baseline")
        self.cali_file_folder = rospack.get_path('duckietown') + "/config/" + self.config + "/calibration/camera_intrinsic/"
    
        self.frame_id = rospy.get_namespace().strip('/') + "/camera_optical_frame"

        self.has_published = False
        self.pub_img= rospy.Publisher("~image/raw", Image,queue_size=1)

        # Create service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",SetCameraInfo,self.cbSrvSetCameraInfo)

        # Setup PiCamera
        self.stream = io.BytesIO()
        self.camera = PiCamera()
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w,self.res_h)

        self.is_shutdown = False
        # Setup timer
        self.gen = self.grabAndPublish(self.stream,self.pub_img)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        self.camera.capture_sequence(self.gen,'yuv',use_video_port=True,splitter_port=0)
        self.camera.close()
        rospy.sleep(rospy.Duration.from_sec(2.0))
        rospy.loginfo("[%s] Capture Ended." %(self.node_name))

    def grabAndPublish(self,stream,publisher):
        while True: #TODO not being triggere correctly when shutting down.
            if self.is_shutdown:
                rospy.loginfo("[%s] Stopping stream...." %(self.node_name))
                # raise StopIteration
                return

            yield stream

            # Grab image from stream
            stamp = rospy.Time.now()
            stream.seek(0)
            stream_data = stream.getvalue()
            
            # Turn strings into numpy array and image_msg
            t1 = time.time()
            #cv_image = np.fromstring(stream_data, dtype=np.uint8)
            cv_image = self.yuv_bytes_to_rgb(stream_data)
            #cv_image = bytes_to_yuv(stream_data, (640, 480)) 
            t2 = time.time()
            print 'string to numpy array%.6f'%(t2-t1)
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
            #t3 = time.time()
            #print 'numpy to message %.6f'%(t3 - t2)
            #cv_tmp = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            #print 'message to numpy %.6f'%(time.time() - t3)


            #image_msg = Image()
            #image_msg.data = stream_data

            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            publisher.publish(image_msg)
                        
            # Clear stream
            stream.seek(0)
            stream.truncate()
            
            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %(self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def yuv_bytes_to_rgb(self, data):
        t1 = time.time()
        y_len = self.res_w * self.res_h
        uv_len = (self.res_w // 2) * (self.res_h // 2)
        
        # Separate out the Y, U, and V values from the array
        a = np.fromstring(data, dtype=np.uint8)
        Y = a[:y_len]
        U = a[y_len:-uv_len]
        V = a[-uv_len:]
        
        # Reshape the values into two dimensions, and double the size of the
        # U and V values (which only have quarter resolution in YUV4:2:0)
        Y = Y.reshape((self.res_h, self.res_w))
        U = U.reshape((self.res_h // 2, self.res_w // 2)).repeat(2, axis=0).repeat(2, axis=1)
        V = V.reshape((self.res_h // 2, self.res_w // 2)).repeat(2, axis=0).repeat(2, axis=1)
        
        t2 = time.time()
        print 'Bytes to YUV: %.6f'%(t2-t1)
        # Stack the channels together and crop to the actual resolution
        YUV = np.dstack((Y, U, V))[:self.res_h, :self.res_w, :].astype(np.float)
        YUV[:, :, 0]  = YUV[:, :, 0]  - 16   # Offset Y by 16
        YUV[:, :, 1:] = YUV[:, :, 1:] - 128  # Offset UV by 128
        
        # YUV conversion matrix from ITU-R BT.601 version (SDTV)
        #              Y       U       V
        M = np.array([[1.164,  0.000,  1.596],    # R
              [1.164, -0.392, -0.813],    # G
              [1.164,  2.017,  0.000]])   # B
        
        t3 = time.time()
        # Take the dot product with the matrix to produce RGB output, clamp the
        # results to byte range and convert to bytes
        RGB = YUV.dot(M.T).astype(np.uint8)
        print 'YUV to RGB: %.6f'%(time.time()-t3)
        return RGB

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." %(self.node_name))
        # self.camera.stop_recording(splitter_port=0)
        # rospy.sleep(rospy.Duration.from_sec(2.0))
        self.camera.close()
        rospy.sleep(rospy.Duration.from_sec(2.0))
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

    def signal_handler(self, signal, frame):
        print "==== Ctrl-C Pressed ==== "
        self.is_shutdown = True

    def cbSrvSetCameraInfo(self,req):
        # TODO: save req.camera_info to yaml file
        rospy.loginfo("[cbSrvSetCameraInfo] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info,filename)
        response.status_message = "Write to %s" %filename #TODO file name
        return response

    def saveCameraInfo(self, camera_info_msg, filename):
        # Convert camera_info_msg and save to a yaml file
        rospy.loginfo("[saveCameraInfo] filename: %s" %(filename))
        file = open(filename, 'w')

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
        'image_height': camera_info_msg.height,
        'camera_name': rospy.get_name().strip("/"), #TODO check this
        'distortion_model': camera_info_msg.distortion_model,
        'distortion_coefficients': {'data': camera_info_msg.D, 'rows':1, 'cols':5},
        'camera_matrix': {'data': camera_info_msg.K, 'rows':3, 'cols':3},
        'rectification_matrix': {'data': camera_info_msg.R, 'rows':3, 'cols':3},
        'projection_matrix': {'data': camera_info_msg.P,'rows':3, 'cols':4}}
        
        rospy.loginfo("[saveCameraInfo] calib %s" %(calib))

        try:
            rc = yaml.safe_dump(calib, file)
            return True
        except IOError:
            return False

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False,disable_signals=True)
    camera_node = CameraNode()
    signal.signal(signal.SIGINT, camera_node.signal_handler)
    rospy.on_shutdown(camera_node.onShutdown)
    camera_node.startCapturing()
    rospy.spin()
