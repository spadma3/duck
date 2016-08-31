#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
from vehicle_detection_tracking.Detector import Detector
from vehicle_detection_tracking.Tracker import Tracker
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import VehicleDetected, VehicleBoundingBox
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from mutex import mutex


class TLD():
	def __init__(self):
		pos_dist = np.load("/home/ubuntu/TLD/posDist.npy").tolist()
		neg_dist = np.load("/home/ubuntu/TLD/negDist.npy").tolist()
		self.active = True
		self.bridge = CvBridge
		self.Detector = Detector()
		self.Detector.set_posterior(pos_dist,neg_dist)
		self.Tracker = Tracker()
		self.tracking = False
		self.sub_image = rospy.Subscriber("/autopilot/camera_node/image", CompressedImage, self.cbImage, queue_size=1)
		self.pub_image = rospy.Publisher("~image_with_detection", Image, queue_size=1)
		self.pub_vehicle_detected = rospy.Publisher("~vehicle_detected", VehicleDetected, queue_size=1)
		self.pub_vehicle_bbox = rospy.Publisher("~vehicle_bounding_box", VehicleBoundingBox, queue_size=1)
		self.lock = mutex()


	def cbImage(self, image_msg):
		if not self.active:
			return
		thread = threading.Thread(target=self.run,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()
		print "Started subscription"


	def run(self, image_msg):
		if self.lock.testandset():
			vehicle_bounding_box_msg = VehicleBoundingBox()
			vehicle_detected_msg = VehicleDetected()
			try:
				image_cv = self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
			except CvBridgeError as e:
				print e
			image_cv = image_cv[80:200,0:640]
			if not self.tracking:
				print "Start Detecting"
				veh = self.Detector.run(image_cv)
				if veh == None:
					vehicle_detected_msg = False
					self.pub_vehicle_detected.publish(vehicle_detected_msg)
				else:
					vehicle_detected_msg = True
					vehicle_bounding_box_msg.x1 = veh[0]
					vehicle_bounding_box_msg.y1 = veh[1]
					vehicle_bounding_box_msg.x2 = veh[2]
					vehicle_bounding_box_msg.y2 = veh[3]
					cv2.rectangle(image_cv, (veh[0],veh[1]), (veh[2],veh[3]),(255,0,0),1)
					image_msg = self.bridge.cv2_to_imgmsg(image_cv,"bgr8")
					self.pub_vehicle_bbox.publish(vehicle_bounding_box_msg)
					self.pub_vehicle_detected.publish(vehicle_detected_msg)
					self.pub_image.publish(image_msg)
					self.Tracker.initialize(veh,image_cv)
					self.tracking = True
			else:
				veh = self.Tracker.run(image_cv)
				if veh == None:
					self.tracking = False
					vehicle_detected_msg = False
					self.pub_vehicle_detected.publish(vehicle_detected_msg)
				else:
					vehicle_detected_msg = True
					vehicle_bounding_box_msg.x1 = veh[0]
					vehicle_bounding_box_msg.y1 = veh[1]
					vehicle_bounding_box_msg.x2 = veh[2]
					vehicle_bounding_box_msg.y2 = veh[3]
					cv2.rectangle(image_cv, (veh[0],veh[1]), (veh[2],veh[3]),(255,0,0),1)
					image_msg = self.bridge.cv2_to_imgmsg(image_cv,"bgr8")
					self.pub_vehicle_bbox.publish(vehicle_bounding_box_msg)
					self.pub_vehicle_detected.publish(vehicle_detected_msg)
					self.pub_image.publish(image_msg)

			self.lock.unlock()

if __name__ == "__main__":
	rospy.init_node("vehicle_detection_tracking_node")
	vehicle_detection_tracking_node = TLD()
	rospy.spin()