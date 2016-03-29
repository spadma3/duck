#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
import threading

class DecoderNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Thread lock
        self.thread_lock = threading.Lock()

        self.bridge = CvBridge()
        
        self.publish_freq = self.setupParam("~publish_freq",1.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~image/raw",Image,queue_size=1)
        self.last_stamp = rospy.Time.now()        
        self.sub_compressed_img = rospy.Subscriber("~compressed_image",CompressedImage,self.cbImg,queue_size=1)

        # Verbose option 
        self.verbose = rospy.get_param('~verbose')

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImg(self, msg):
        thread = threading.Thread(target=self.processImg, args=(msg, ))
        thread.setDaemon(True)
        thread.start()

    def processImg(self, msg):
        if not self.thread_lock.acquire(False):
            return

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now
        
        # Verbose
        if self.verbose:
            rospy.loginfo("[%s] Latency received = %.3f ms" %(self.node_name, (rospy.get_time()-msg.header.stamp.to_sec()) * 1000.0))

        # time_start = time.time()
        np_arr = np.fromstring(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # time_1 = time.time()
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(img_msg)

        # Verbose
        if self.verbose:
            rospy.loginfo("[%s] Latency sent = %.3f ms" %(self.node_name, (rospy.get_time()-msg.header.stamp.to_sec()) * 1000.0))

        # time_3 = time.time()
        # rospy.loginfo("[%s] Took %f sec to decompress."%(self.node_name,time_1 - time_start))
        # rospy.loginfo("[%s] Took %f sec to conver to Image."%(self.node_name,time_2 - time_1))
        # rospy.loginfo("[%s] Took %f sec to publish."%(self.node_name,time_3 - time_2))

        self.thread_lock.release()

if __name__ == '__main__': 
    rospy.init_node('decoder_low_freq',anonymous=False)
    node = DecoderNode()
    rospy.spin()

