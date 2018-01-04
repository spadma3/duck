#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, VehiclePose, Pose2DStamped

import os
import rospkg
import rospy
import yaml
import time
from twisted.words.protocols.oscar import CAP_SERV_REL
from math import sqrt, sin, cos
from std_msgs.msg import Float32

class VehicleAvoidanceControlNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospack = rospkg.RosPack()
		self.car_vel_pub = rospy.Publisher("~car_vel",
				Float32, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
				BoolStamped, queue_size=1)
		self.subscriber = rospy.Subscriber("~detection",
				BoolStamped, self.callback,  queue_size=1)
		self.sub_vehicle_pose = rospy.Subscriber("~vehicle_pose", VehiclePose, self.cbPose, queue_size=1)
		self.sub_car_cmd = rospy.Subscriber("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)
		self.sub_switch = rospy.Subscriber("~switch", BoolStamped,
				self.cbSwitch, queue_size=1)
		
		self.config	= self.setupParam("~config", "baseline")
		self.cali_file_name = self.setupParam("~cali_file_name", "default")
		rospack = rospkg.RosPack()
		self.cali_file = rospack.get_path('duckietown') + \
				"/config/" + self.config + \
				"/vehicle_detection/vehicle_avoidance_control_node/" +  \
				self.cali_file_name + ".yaml"
		if not os.path.isfile(self.cali_file):
			rospy.logwarn("[%s] Can't find calibration file: %s.\n" 
					% (self.node_name, self.cali_file))
		self.loadConfig(self.cali_file)
		
		self.controllerInitialization()
		

	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value
	
	def loadConfig(self, filename):
		stream = file(filename, 'r')
		data = yaml.load(stream)
		stream.close()
		self.desired_distance = data['desired_distance']
		rospy.loginfo('[%s] desired_distance : %s' % (self.node_name, 
				self.desired_distance))
		self.minimal_distance = data['minimal_distance']
		rospy.loginfo('[%s] minimal_distance : %s' % (self.node_name, 
				self.minimal_distance))
		self.Kp = data['Kp']
		rospy.loginfo('[%s] Kp : %s' % (self.node_name, 
				self.Kp))
		self.Ki = data['Ki']
		rospy.loginfo('[%s] Ki : %s' % (self.node_name, 
				self.Ki))
		self.Kd = data['Kd']
		rospy.loginfo('[%s] Kd : %s' % (self.node_name, 
				self.Kd))
		self.Kp_delta_v = data['Kp_delta_v']
		rospy.loginfo('[%s] Kp_delta_v : %s' % (self.node_name, 
				self.Kp_delta_v))
		
	def cbSwitch(self, switch_msg):
		self.active = switch_msg.data
		
	def controllerInitialization(self):
		self.vehicle_pose_msg_temp = VehiclePose()
		self.vehicle_pose_msg_temp.header.stamp = rospy.Time.now()
		self.time_temp = rospy.Time.now()
		self.v_rel = 0
		self.v = 0
		self.detection = False
		self.v_error_temp = 0
		self.I = 0
		self.v_follower = 0
		self.omega = 0

	def callback(self, data):
		if not self.active:
			return
		
		vehicle_detected_msg_out = BoolStamped()
		vehicle_detected_msg_out.header.stamp = data.header.stamp
		vehicle_detected_msg_out.data = data.data
		self.vehicle_detected_pub.publish(vehicle_detected_msg_out)
		self.detection = data.data
		
		if  not data.data:
			self.I = 0
			
		
	def cbPose(self, vehicle_pose_msg):
		if not self.active:
			return
				
		time = rospy.Time.now()
		Ts = (time - self.time_temp).to_sec()

		self.vehicle_pose_msg_temp.header.stamp = vehicle_pose_msg.header.stamp

		if Ts > 2:
			self.v_rel = 0
			if vehicle_pose_msg.rho.data < self.minimal_distance:
				self.v = 0
			else:
				self.v = self.v_follower
			self.vehicle_pose_msg_temp = vehicle_pose_msg
			self.v_error_temp = 0
			self.I = 0
		else:
			self.v_rel = (vehicle_pose_msg.rho.data - self.vehicle_pose_msg_temp.rho.data)/Ts
			v_leader = self.v_follower + self.v_rel
			delta_v = (vehicle_pose_msg.rho.data - self.desired_distance)/Ts * self.Kp_delta_v
			v_des = v_leader + delta_v
						
			v_error = v_des - self.v_follower

			self.P = self.Kp*v_error
			self.I = self.I + self.Ki * (v_error + self.v_error_temp)/2.0*Ts 
			self.D = self.Kd * (v_error + self.v_error_temp)/Ts 
			self.v = self.P + self.I + self.D
			
			if self.v < 0 or vehicle_pose_msg.rho.data < self.minimal_distance:
				self.v = 0
			
			self.v_error_temp = v_error
			self.v_temp = self.v
			self.vehicle_pose_msg_temp = vehicle_pose_msg
			
		self.time_temp = time
						
	def cbCarCmd(self, car_cmd_msg):
		if not self.active:
			return
# 		car_cmd_msg_current = Twist2DStamped()
# 		car_cmd_msg_current = car_cmd_msg
# 		car_cmd_msg_current.header.stamp = rospy.Time.now()

		if self.detection:
			car_vel_msg = self.v
# 			car_cmd_msg_current.v = self.v
# 			if self.v == 0:
# 				car_cmd_msg_current.omega = 0
		else:
			car_vel_msg = car_cmd_msg.v

# 		self.v_follower = car_cmd_msg_current.v	
# 		self.car_cmd_pub.publish(car_cmd_msg_current)
		self.v_follower = car_vel_msg
		self.car_vel_pub.publish(car_vel_msg)
   
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()

