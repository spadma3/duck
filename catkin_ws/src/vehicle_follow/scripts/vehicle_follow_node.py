#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import Twist2DStamped, Pose2DPolarStamped
from sensor_msgs.msg import Joy


class VehicleFollow(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pose_2d_polar = Pose2DPolarStamped()

        self.car_cmd_msg = Twist2DStamped()

        self.pub_counter = 1

        # Setup parameters
        self.reference_distance = self.setup_parameter("~reference_distance", 0.15)
        self.reference_angle = self.setup_parameter("~reference_angle", 0.0)
        self.k_follow = self.setup_parameter("~k_follow", 1.0)  # Linear velocity
        self.k_heading = self.setup_parameter("~k_heading", 1.0)  # P gain for theta

        self.heading_thres = self.setup_parameter("~heading_thres", math.pi/4)  # Maximum desired heading
        self.max_speed = self.setup_parameter("~max_speed", 0.4)
        self.max_heading = self.setup_parameter("~max_heading", 0.2)

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_pose = rospy.Subscriber("~pose", Pose2DPolarStamped, self.cb_pose, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.update_params)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.get_gains_event)
        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def update_params(self, event):
        self.reference_distance = rospy.get_param("~reference_distance")
        self.reference_angle = rospy.get_param("~reference_angle")
        self.max_speed = rospy.get_param("~max_speed")
        self.max_steer = rospy.get_param("~max_steer")

    def get_gains_event(self, event):
        params_old = (self.reference_distance, self.reference_angle, self.k_follow, self.k_heading, self.heading_thres, self.max_speed, self.max_heading)

        reference_distance = rospy.get_param("~reference_distance")
        reference_angle = rospy.get_param("~reference_angle")
        k_follow = rospy.get_param("~k_follow")
        k_heading = rospy.get_param("~k_heading")
        heading_thres = rospy.get_param("~heading_thres")
        max_speed = rospy.get_param("~max_speed")
        max_heading = rospy.get_param("~max_heading")

        params_new = (reference_distance, reference_angle, k_follow, k_heading, heading_thres, max_speed, max_heading)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." % self.node_name)
            rospy.loginfo(
                "old gains, reference_distance %f, reference_angle %f, k_follow %f, k_heading %f, heading_thres %f, max_speed %f, max_heading %f" % params_old)
            rospy.loginfo(
                "new gains, reference_distance %f, reference_angle %f, k_follow %f, k_heading %f, heading_thres %f, max_speed %f, max_heading %f" % params_new)
            self.reference_distance = reference_distance
            self.reference_angle = reference_angle
            self.k_follow = k_follow
            self.k_heading = k_heading
            self.heading_thres = heading_thres
            self.max_speed = max_speed
            self.max_heading = max_heading

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_pose.unregister()

        # Send stop command to car command switch
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(car_control_msg)
        rospy.sleep(0.5)  # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

    def cb_pose(self, pose_2d_polar_msg):
        self.pose_2d_polar = pose_2d_polar_msg

        # copy message header over:
        self.car_cmd_msg.header = pose_2d_polar_msg.header

        # Following Error Calculation
        following_error = self.pose_2d_polar.rho - self.reference_distance

        self.car_cmd_msg.v = self.k_follow * following_error

        # Heading Error Calculation
        heading_error = self.pose_2d_polar.psi - self.reference_angle
        self.car_cmd_msg.omega = self.k_heading * heading_error

        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        self.pub_car_cmd.publish(self.car_cmd_msg)

        # debugging
        self.pub_counter += 1
        if self.pub_counter % 50 == 0:
            self.pub_counter = 1
            print "vehicle_follow publish"
            print car_cmd_msg


if __name__ == "__main__":
    rospy.init_node("vehicle_follow_node", anonymous=False)
    lane_supervisor_node = VehicleFollow()
    rospy.spin()
