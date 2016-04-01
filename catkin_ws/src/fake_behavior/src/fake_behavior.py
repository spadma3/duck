#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('fake_behavior')
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from std_msgs.msg import String
from duckietown_msgs.msg import FSMState, IntersectionDetection, TrafficLightDetection, VehicleDetection
from nav_msgs.msg import Odometry
from random import randint
import std_msgs.msg


from ros import rostopic, rosgraph
import sys
import math


# Get the list of all intersections
def getListOfIntersections ():
	list_of_intersections=[]
	
	# calls a service to obtain all the models in the simulation
	WorldService = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
	properties = WorldService()
	for model in properties.model_names:
		if 'road_inter' in model or 'road_empty' in model:
			list_of_intersections.append(model)
	
	return list_of_intersections

# Get the list of all traffic lights
def getListOfTrafficLights ():
	list_of_tlights=[]
	
	# calls a service to obtain all the models in the simulation
	WorldService = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
	properties = WorldService()
	for model in properties.model_names:
		if 'Semaforo' in model:
			list_of_tlights.append(model)
	
	return list_of_tlights


class odometry_class (object):
	
	def __init__ (self, robot_name):
		rospy.Subscriber("/"+robot_name+"/odom", Odometry, self.odomCallback)
		self.lastOdomX = 0.0
		self.lastOdomY = 0.0
		self.is_moving = False

	def odomCallback (self, data):
		x = data.pose.pose.position.x
		y = data.pose.pose.position.y
		
		if abs(x - self.lastOdomX) > 0.005 or abs(y - self.lastOdomY) > 0.005:
			self.is_moving = True
			print "Robot moving"
		else: # only when show stopped for more than 10 steps we declare it stopped
			self.is_moving = False
		
		self.lastOdomX = x
		self.lastOdomY = y
		
		return


class publisher_class (object):
	
	RobotDistService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	
	def __init__ (self, robot_name):
		self.this_robot = robot_name
		# defining the publishers
		self.mode_pub = rospy.Publisher('/' + self.this_robot + '/mode', FSMState, queue_size=10)
		self.intersection_pub = rospy.Publisher('/' + self.this_robot + '/intersection_detection', IntersectionDetection, queue_size=10)
		self.traffic_pub = rospy.Publisher('/' + self.this_robot + '/traffic_light_detection', TrafficLightDetection, queue_size=10)
		self.right_pub = rospy.Publisher('/' + self.this_robot + '/right_vehicle_detection', VehicleDetection, queue_size=10)
		self.opposite_pub = rospy.Publisher('/' + self.this_robot + '/opposite_vehicle_detection', VehicleDetection, queue_size=10)
		self.intersections_list = getListOfIntersections()
		self.trafficlights_list = getListOfTrafficLights()
		
		self.odom = odometry_class (robot_name)
		self.list_of_robots=[]
		self.list_of_robotmodes=[]
		for i in range(0, 3):
			self.list_of_robotmodes.append(VehicleDetection())
		
		self.intersectionStatus = IntersectionDetection(type=IntersectionDetection.NONE)

	# Get a list of all the other robots in the simulation by checking the odom topic
	def getListOfRobots (self):
		
		this_robot_exists = False
		topics = [t for t, _ in rosgraph.Master('/fake_behav_'+str(self.this_robot)).getPublishedTopics('') if not t.startswith('/rosout')]
		for topic_name in topics:
			# if the topic contains odm, it means that it is a robot
			if 'odom' in topic_name:
				robot_name = topic_name.split('/')[1].split('odom')[0].strip('/')
				# if the name of the robot is different from ours, is another robot in the simulation
				#print "Checking robot names:"+str(robot_name)+" -- "+str(self.this_robot)
				if robot_name != self.this_robot:
					# if this new robot has not been detected previously, it means that a new robot has been spawned and need to include and attach to its topics
					if robot_name in self.list_of_robots:
						pass
					else:
						self.list_of_robots.append(robot_name)
						self.connect_to_robot_model(robot_name)
				else:
					this_robot_exists = True
		
		if not this_robot_exists:
			rospy.logerr ("ERROR: robot "+str(self.this_robot)+" for this fake_behavior node does not exists")
			rospy.sleep(5.)
			
		return self.list_of_robots

	def get_detection_value(self, mode):
		if mode == FSMState.LANE_FOLLOWING:
			return VehicleDetection(detection=VehicleDetection.SIGNAL_A)
		if mode == FSMState.COORDINATION:
			return VehicleDetection(detection=VehicleDetection.SIGNAL_B)
		return VehicleDetection(detection=VehicleDetection.SIGNAL_C)

	def modeCallback_robot1(self, data):
		self.list_of_robotmodes[0] = self.get_detection_value(data.state)
		return
	
	def modeCallback_robot2(self, data):
		self.list_of_robotmodes[1] = self.get_detection_value(data.state)
		return
	
	def modeCallback_robot3(self, data):
		self.list_of_robotmodes[2] = self.get_detection_value(data.state)
		return

	# tthis functiton is called everytime a new robot s spawned
	def connect_to_robot_model(self, robot_name):
		nrobot = len(self.list_of_robots)
		
		print "number of other robots="+str(nrobot)
		if nrobot <= 3:
			if nrobot == 1:
				rospy.Subscriber("/"+robot_name+"/mode", FSMState, self.modeCallback_robot1)
			if nrobot == 2:
				rospy.Subscriber("/"+robot_name+"/mode", FSMState, self.modeCallback_robot2)
			if nrobot == 3:
				rospy.Subscriber("/"+robot_name+"/mode", FSMState, self.modeCallback_robot3)
		else:
			rospy.logerr("ERROR: this fake_behavior node can only work up to three robot vehicles. This simulation has more.")

	# Get a distance between two objects of the simulation
	#rosservice call /gazebo/get_model_state obj1 obj2
	def getDistance(self, obj1, obj2):
		
		dist = 1000.0
		try:
			dxy = self.RobotDistService(obj1, obj2)
			dist = math.sqrt((dxy.pose.position.x)**2 + (dxy.pose.position.y)**2)
			print "Calculated dist between "+ str(obj1) + " and "+ str(obj2) + " is " + str(dist)
		except rospy.ServiceException as exc:
			print("Fake_coordinator for " + str(this_robot) + " did not process request: " + str(exc))
	
		return dist, dxy.pose.position.x, dxy.pose.position.y

	def publish_fake_values(self):
		self.publish_mode()
		self.publish_vehicle_detection()
		self.publish_intersection_detection()

	def publish (self, publisher, msg):
		h = std_msgs.msg.Header()	
		h.stamp = rospy.Time.now()
		msg.header = h
		publisher.publish(msg)

	def publish_mode(self):
		
		# default mode is LANE_FOLLOWING
		mode = FSMState(state=FSMState.LANE_FOLLOWING)
		
		# if veh is at an intersection (either with or without traffic light)
		if self.intersectionStatus.type == IntersectionDetection.STOP or self.intersectionStatus.type == IntersectionDetection.TRAFFIC_LIGHT:
			if self.odom.is_moving:
				# if moving, then it is navigating the intersection
				mode.state = FSMState.INTERSECTION_CONTROL
			else:
				# if not moving, then it is trying to coordinate
				mode.state = FSMState.COORDINATION
		
		# publish msg with mode
		print "Mode:"+ str(mode.state)
		self.publish(self.mode_pub, mode)

	def getRobotStatus(self,robot):
		status = VehicleDetection.NO_CAR
		for n in range(0, len(self.list_of_robots)):
			if self.list_of_robots[n] == robot:
				status =self.list_of_robotmodes[n]
				break
		
		return status

	def publish_vehicle_detection (self):
		
		right_status=VehicleDetection(detection=VehicleDetection.NO_CAR)
		front_status=VehicleDetection(detection=VehicleDetection.NO_CAR)
		# we need to calculate list of available robots all the time
		# because new ones could have been spawned
		robots_list = self.getListOfRobots()
		for another_robot in robots_list:
			dist, distx, disty = self.getDistance("duckiebot_"+str(self.this_robot), "duckiebot_"+str(another_robot)+"::base_footprint")
			print "Between "+str(self.this_robot)+" and "+str(another_robot)+" is "+str(distx)+","+str(disty)
			# check for a vehicle on the right
			if abs(distx) > 1.06 and abs(distx) < 3.86 and abs(disty) > 4.26 and abs(disty) < 6.72:
				right_status = self.getRobotStatus (another_robot)
				print str(another_robot)+ " is on the right of "+str(self.this_robot)
				
			# check for a vehicle in front left
			if abs(distx) > 7.3 and abs(distx) < 8.5 and abs(disty) > 1.35 and abs(disty) < 4.6:
				front_status = self.getRobotStatus (another_robot)
				print str(another_robot)+ " is on the front of "+str(self.this_robot)

		print "Right status:"+str(right_status.detection)
		self.publish (self.right_pub, right_status)
		print "Front status:"+str(front_status.detection)
		self.publish(self.opposite_pub, front_status)
	
	def publish_intersection_detection (self):
		self.intersectionStatus.type = IntersectionDetection.NONE
		tlight_status = TrafficLightDetection(color=TrafficLightDetection.NONE)
		
		for intersection in self.intersections_list:
			dist = self.getDistance("duckiebot_"+str(self.this_robot), intersection + '::link')
			if dist < 4.48:
				self.intersectionStatus.type = IntersectionDetection.STOP
				print str(self.this_robot)+" is at intersection "+str(intersection)
				for tlight in self.trafficlights_list:
					dist = self.getDistance("duckiebot_"+str(self.this_robot), tlight + '::link')
					if dist < 4.48:
						self.intersectionStatus.type = IntersectionDetection.TRAFFIC_LIGHT
						tlight_status = self.getTrafficLightStatus(tlight)
						print str(self.this_robot)+" is at traffic light "+str(tlight)
						break
				break
		
		self.publish(self.traffic_pub, tlight_status)
		self.publish(self.intersection_pub, self.intersectionStatus)

	def getTrafficLightStatus(self,tlight):
		
		# we just generate a random value for the traffic light color
		# otherwise is too complex
		status = TrafficLightDetection(color=TrafficLightDetection.NONE)
		n = randint(0,3)
		status.color = n
		
		return status
		
	def start(self):
		r = rospy.Rate(1) # 1hz
		while not rospy.is_shutdown():
			publisher.publish_fake_values()
			r.sleep()

# Main function.
if __name__ == '__main__':

	if len(sys.argv) < 2:
		print("usage: fake_behavior.py <robot_name>")
	else:
		# Get this robot name
		this_robot= sys.argv[1]
		
		rospy.init_node('fake_behavior_node')
		rospy.wait_for_service('/gazebo/get_model_state')
	
		rospy.sleep(1.0)
		publisher = publisher_class(this_robot)
		
		publisher.start()
		
		