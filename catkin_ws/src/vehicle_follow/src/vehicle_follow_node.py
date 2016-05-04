#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import Twist2DStamped, LanePose, StopLineReading
from sensor_msgs.msg import Joy


class VehicleFollow(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = LanePose()
        self.car_control_lane = Twist2DStamped()
        self.car_control_joy = Twist2DStamped()
        self.safe = True
        self.in_lane = True
        self.at_stop_line = False
        self.stop = False

        # Params:
        self.max_cross_track_error = self.setup_parameter("~max_cross_track_error", 0.1)
        self.max_heading_error = self.setup_parameter("~max_heading_error", math.pi / 4)
        self.min_speed = self.setup_parameter("~min_speed", 0.1)
        self.max_speed = self.setup_parameter("~max_speed", 0.3)
        self.max_steer = self.setup_parameter("~max_steer", 0.2)

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_safe = rospy.Publisher("~safe", Bool, queue_size=1)

        # Subscriptions
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.cb_lane_pose, queue_size=1)
        self.sub_lane_control = rospy.Subscriber("~car_cmd_lane", Twist2DStamped, self.cb_lane_control, queue_size=1)
        self.sub_joy_control = rospy.Subscriber("~car_cmd_joy", Twist2DStamped, self.cb_joy_control, queue_size=1)
        self.sub_at_stop_line = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cb_stop_line, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.update_params)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def update_params(self, event):
        self.max_cross_track_error = rospy.get_param("~max_cross_track_error")
        self.max_heading_error = rospy.get_param("~max_heading_error")
        self.max_speed = rospy.get_param("~max_speed")
        self.max_steer = rospy.get_param("~max_steer")

    def cb_stop_line(self, stop_line_msg):
        if not stop_line_msg.at_stop_line:
            self.at_stop_line = False
            self.stop = False
        else:
            if not self.at_stop_line:
                self.at_stop_line = True
                self.stop = True
                rospy.sleep(2)
                self.stop = False

    def cb_lane_pose(self, lane_pose_msg):
        self.in_lane = lane_pose_msg.in_lane
        self.lane_reading = lane_pose_msg
        cross_track_err = math.fabs(lane_pose_msg.d)
        heading_err = math.fabs(lane_pose_msg.phi)
        if cross_track_err > self.max_cross_track_error or heading_err > self.max_heading_error:
            self.safe = False
        else:
            self.safe = True
        self.pub_safe.publish(self.safe)

    def cb_lane_control(self, lane_control_msg):
        self.car_control_lane = lane_control_msg

    def cb_joy_control(self, joy_control_msg):
        self.car_control_joy = joy_control_msg
        car_cmd_msg = self.merge_joy_and_lane_control()
        self.pub_car_cmd.publish(car_cmd_msg)

    def merge_joy_and_lane_control(self):
        car_cmd_msg = Twist2DStamped()
        if self.stop:
            rospy.loginfo("[PA] stopped at stop line")
            car_cmd_msg.v = 0
            car_cmd_msg.omega = 0
        elif self.safe:  # or not self.in_lane:
            rospy.loginfo("[PA] in safe mode")
            self.car_control_joy.v = min(self.car_control_joy.v, self.max_speed)
            self.car_control_joy.omega = np.clip(self.car_control_joy.omega, -self.max_steer, self.max_steer)
            car_cmd_msg = self.car_control_joy
            car_cmd_msg.header.stamp = self.car_control_joy.header.stamp
        else:
            rospy.loginfo("[PA] not safe - merge control inputs")
            car_control_merged = Twist2DStamped()
            if abs(self.car_control_joy.v) < self.min_speed:
                # sets the speeds to 0:
                car_control_merged.v = 0.0
                car_control_merged.omega = 0.0
            else:
                # take the speed from the joystick:
                car_control_merged.v = min(self.car_control_joy.v, self.max_speed)
                # take the omega from the lane controller:
                car_control_merged.omega = self.car_control_lane.omega
            car_cmd_msg = car_control_merged
            car_cmd_msg.header.stamp = self.car_control_joy.header.stamp
        return car_cmd_msg


if __name__ == "__main__":
    rospy.init_node("lane_supervisor", anonymous=False)
    lane_supervisor_node = VehicleFollow()
    rospy.spin()
