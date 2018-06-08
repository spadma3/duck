#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32, Bool
from sensor_msgs.msg import Image, CameraInfo
from duckietown_msgs.msg import Pose2DStamped, Twist2DStamped
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from visual_odometry import PinholeCamera, VisualOdometry
# DVisual Odometry node
# Author: Gianmarco Bernasconi/Julien Kindle
# Inputs: ~Image/Image - Input raw image
# Outputs: ~pose/Int32 - The output pose

class VOEstimator(object):
	def __init__(self):
		self.node_name = "Visual Odometry Node"
		self.running = False
		self.pinhole_cam = PinholeCamera(640.0, 480.0, 347.5, 338.46, 313.8, 263.9)
		self.bridge = CvBridge()

		self.setupParams()

		# Subscribers
		self.sub_img = rospy.Subscriber("~image", Image, self.cbImage, queue_size=1)
		self.sub_start = rospy.Subscriber("~start_estimation", Bool, self.cbStartEstimation, queue_size=1)
		self.sub_stop = rospy.Subscriber("~stop_estimation", Bool, self.cbStopEstimation, queue_size=1)
		self.sub_cam_info = rospy.Subscriber("~camera_info", CameraInfo, self.cbCameraInfo, queue_size=1)
		self.sub_for_kin = rospy.Subscriber("~for_kin_node_velocities", Twist2DStamped, self.cbVelocities, queue_size=1)

		## Publisher
		self.pub_estimation = rospy.Publisher("~estimation", Pose2DStamped, queue_size=1)
		self.pub_pose_viz = rospy.Publisher("~pose_visualization", PointStamped, queue_size=1)

		self.VisualOdometryEstimator = None
		self.frame_id = -2
		self.last_pos = np.array([0,0])

		## update Parameters timer
		self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


	######## Callback functions begin ########
	def cbVelocities(self, msg):
		if self.VisualOdometryEstimator is not None:
			self.VisualOdometryEstimator.velocity = msg.v

	def cbCameraInfo(self, msg):
		K = msg.K
		self.cam = PinholeCamera(msg.width, msg.height, K[0], K[4], K[2], K[5])
	def cbImage(self, msg):
		if not self.running: return

		self.frame_id = self.frame_id + 1
		try:
			img = self.bridge.imgmsg_to_cv2(msg, "mono8")
		except CvBridgeError as e:
		  	print(e)
			return

		# Update our estimation
		self.VisualOdometryEstimator.update(img)

		if self.frame_id <= 0: return
		self.estimatePosition()

	def cbStopEstimation(self, msg):
		if not msg.data: return
		self.running = False
		self.VisualOdometryEstimator = None

	def cbStartEstimation(self, msg):
		if not msg.data: return
		self.running = True

		self.VisualOdometryEstimator = VisualOdometry(self.pinhole_cam, self.min_features)
		self.frame_id = -2

	######## Callback functions end ########

	######## Functionality begin ########
	def estimatePosition(self):
		cur_t = self.VisualOdometryEstimator.cur_t
		x, y, z = cur_t[0], cur_t[1], cur_t[2]
		x2,y2,z2 = x,y,z
		# Rotate around axis (since camera is tilted)
		a = -np.pi/36
		vec = np.array([x,y,z])
		R = np.matrix([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
		vec_rot = R.dot(vec)

		# Using coordinate system: x initial driving direction, y 90deg right of init pose
		y,_,x = -float(vec_rot[0]),float(vec_rot[1]),float(vec_rot[2])

		direction = np.array([x,y]) - self.last_pos
		theta = np.arctan2(direction[1],direction[0])

		# Publish obtained position
		pose_msg = Pose2DStamped()
		pose_msg.x = x
		pose_msg.y = y
		pose_msg.theta = theta

		self.pub_estimation.publish(pose_msg)

		pose_msg2 = PointStamped()
		pose_msg2.point.x = x
		pose_msg2.point.y = y
		pose_msg2.point.z = 0.0
		pose_msg2.header.frame_id = "base_link"
		self.pub_pose_viz.publish(pose_msg2)

	######## Functionality end ########

	######## Parameter functions begin ########
	def setupParams(self):
		self.min_features = self.setupParam("~min_features", 50)
		self.intersection_speed = self.setupParam("~intersection_speed", 0.1)

	def updateParams(self,event):
		self.min_features = rospy.get_param("~min_features")
		self.intersection_speed = rospy.get_param("~intersection_speed")

	def setupParam(self,param_name,default_value):
		value = rospy.get_param(param_name,default_value)
		rospy.set_param(param_name,value) #Write to parameter server for transparancy
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	### Parameter functions end ###

if __name__ == '__main__':
	rospy.init_node('visual_odometry_node', anonymous=False)
	voestimator = VOEstimator()
	rospy.spin()
