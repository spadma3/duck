#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import Twist2DStamped, VehiclePose
from sensor_msgs.msg import Joy


class VehicleFollow(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.vehicle_pose_last = VehiclePose()

        self.car_cmd_msg = Twist2DStamped()

        self.pub_counter = 1

        # Setup parameters
        self.dist_ref = self.setup_parameter("~dist_ref", 0.15)
        self.head_ref = self.setup_parameter("~head_ref", 0.0)
        self.k_follow = self.setup_parameter("~k_follow", 1.0)  # Linear velocity
        self.k_heading = self.setup_parameter("~k_heading", 1.0)  # P gain for theta

        self.head_thres = self.setup_parameter("~head_thres", math.pi / 4)  # Maximum desired heading
        self.max_speed = self.setup_parameter("~max_speed", 0.4)
        self.max_heading = self.setup_parameter("~max_heading", 0.2)
        self.deadspace_speed = self.setup_parameter("~deadspace_speed", 0.05)
        self.deadspace_heading = self.setup_parameter("~deadspace_heading", 0.2)

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_target_pose = rospy.Subscriber("~target_pose", VehiclePose, self.cb_pose, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.update_params_event)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def update_params_event(self, event):
        params_old = (self.dist_ref, self.head_ref, self.k_follow, self.k_heading, self.head_thres, self.max_speed, self.max_heading)

        dist_ref = rospy.get_param("~dist_ref")
        head_ref = rospy.get_param("~head_ref")
        k_follow = rospy.get_param("~k_follow")
        k_heading = rospy.get_param("~k_heading")
        head_thres = rospy.get_param("~head_thres")
        max_speed = rospy.get_param("~max_speed")
        max_heading = rospy.get_param("~max_heading")
        deadspace_speed = rospy.get_param("~deadspace_speed")
        deadspace_heading = rospy.get_param("~deadspace_heading")

        params_new = (dist_ref, head_ref, k_follow, k_heading, head_thres, max_speed, max_heading, deadspace_speed, deadspace_heading)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." % self.node_name)
            rospy.loginfo(
                "old: dist_ref %f, head_ref %f, k_follow %f, k_heading %f, head_thres %f, max_speed %f,"
                " max_heading %f, deadspace_speed %f , deadspace_heading %f " % params_old)
            rospy.loginfo(
                "new: dist_ref %f, head_ref %f, k_follow %f, k_heading %f, head_thres %f, max_speed %f, "
                " max_heading %f, deadspace_speed %f , deadspace_heading %f" % params_new)
            self.dist_ref = dist_ref
            self.head_ref = head_ref
            self.k_follow = k_follow
            self.k_heading = k_heading
            self.head_thres = head_thres
            self.max_speed = max_speed
            self.max_heading = max_heading
            self.deadspace_speed = deadspace_speed
            self.deadspace_heading = deadspace_heading

    def stop_vehicle(self):
        self.car_cmd_msg.v = 0.0
        self.car_cmd_msg.omega = 0.0
        self.pub_car_cmd.publish(self.car_cmd_msg)

    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_target_pose.unregister()

        # Send stop command to car command switch
        self.stop_vehicle()

        rospy.sleep(0.5)  # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" % self.node_name)

    def cb_pose(self, vehicle_pose_msg):

        # copy message header over:
        self.car_cmd_msg.header = vehicle_pose_msg.header

        if not vehicle_pose_msg.detection.data:
            # it stops if it doesnt see
            self.stop_vehicle()
            # keep following last command?
        else:
            # Following Error Calculation
            following_error = vehicle_pose_msg.rho.data - self.dist_ref
            self.car_cmd_msg.v = self.k_follow * following_error

            if self.car_cmd_msg.v > self.max_speed:
                self.car_cmd_msg.v = self.car_cmd_msg.v
            if self.car_cmd_msg.v < - self.max_speed:
                self.car_cmd_msg.v = - self.max_speed
            elif abs(self.car_cmd_msg.v) < self.deadspace_speed:
                self.car_cmd_msg.v = 0.0

            # Heading Error Calculation
            # ToDo try an integrator
            heading_error = vehicle_pose_msg.theta.data - self.head_ref

            self.car_cmd_msg.omega = self.k_heading * heading_error

            if self.car_cmd_msg.omega > self.max_heading:
                self.car_cmd_msg.omega = self.max_heading
            elif self.car_cmd_msg.omega < -self.max_heading:
                self.car_cmd_msg.omega = -self.max_heading
            elif abs(self.car_cmd_msg.omega) < self.deadspace_heading:
                self.car_cmd_msg.omega = 0.0
            # ToDo: what to do with vehicle_pose_msg.psi.data?

            # Publish control message
            self.pub_car_cmd.publish(self.car_cmd_msg)

        self.vehicle_pose_last = vehicle_pose_msg

        # # debugging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "vehicle_follow publish"
        #     print self.car_cmd_msg


if __name__ == "__main__":
    rospy.init_node("vehicle_follow_node", anonymous=False)
    lane_supervisor_node = VehicleFollow()
    rospy.spin()
