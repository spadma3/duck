#!/usr/bin/env python

import rospy
import math
import json
from numbers import Number
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from std_msgs.msg import String

CONTROLLER_TIMEOUT_IN_SEC = 5
CONTROLLER_FREQUENCY_IN_HZ = 100
CMD_EXPECTED_KEYS = [
    'forward_speed',
    'turn_speed'
]

class BlocklyWheelController(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo( "[%s] Initializing "%(self.node_name,) )
        # setup parameters
        self.v_gain = self.setup_param("~speed_gain", 0.41)
        self.omega_gain = self.setup_param("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setup_param("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setup_param("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setup_param("~simulated_vehicle_length", 0.18)
        self.json_cmd_topic = self.setup_param("~json_cmd_topic", "/blockly_drive_json_cmd")
        # create timer
        self.last_pub_cmd = None
        self.last_pub_time = rospy.Time.now().secs
        # create publisher || TODO: hard-coded topic
        self.pub_car_cmd = rospy.Publisher("joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
        # create subscriber
        self.sub_json = rospy.Subscriber(self.json_cmd_topic, String, self.json_msg_cb, queue_size=1)

    def setup_param(self,param_name,default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo( "[%s] %s = %s " % ( self.node_name,param_name,value ) )
        return value

    def json_msg_cb(self, string_msg):
        print 'Recevied "%s"' % string_msg.data
        cmd = json.loads( string_msg.data )
        # make sure all the necessary keys are present || TODO: errors?
        if len(set(CMD_EXPECTED_KEYS).difference(set(cmd.keys()))) > 0: return;
        # make sure all the values are numbers || TODO: errors?
        valid_cmd = {}
        for key in CMD_EXPECTED_KEYS:
            if not isinstance(cmd[key], Number): return;
            valid_cmd[key] = float(cmd[key]) / 100.0
        # perform command
        print valid_cmd
        self.last_pub_time = rospy.Time.now().secs
        self.publish_control( valid_cmd['forward_speed'], valid_cmd['turn_speed'] )

    def publish_control(self, forward_speed, turn_speed):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = forward_speed * self.v_gain
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            steering_angle = turn_speed * self.steer_angle_gain
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = turn_speed * self.omega_gain
        self.last_pub_cmd = car_cmd_msg
        self.pub_car_cmd.publish(car_cmd_msg)

    def start(self):
        rate = rospy.Rate(CONTROLLER_FREQUENCY_IN_HZ)
        while not rospy.is_shutdown():
            if( rospy.Time.now().secs - self.last_pub_time < CONTROLLER_TIMEOUT_IN_SEC ):
                # keep going
                if self.last_pub_cmd is not None: self.pub_car_cmd.publish(self.last_pub_cmd)
            else:
                # stop vehicle
                self.publish_control(0.0, 0.0)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("blockly_wheel_controller", anonymous=False)
    controller = BlocklyWheelController()
    controller.start()
