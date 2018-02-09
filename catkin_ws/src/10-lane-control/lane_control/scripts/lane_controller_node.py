#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose
import os, imp, time

class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # Load files for HW-Exercises
        exercise_txt = os.environ['CSII_EXERCISE']
        exercise = self.get_num(exercise_txt)
        subexercise = self.get_txt(exercise_txt)

        duckietown_root = os.environ['DUCKIETOWN_ROOT']
        ex_path = "/CSII/Exercises/HWExercise" + str(exercise) + "/controller-" + str(exercise_txt) + ".py"
        template_src = imp.load_source('module.name', duckietown_root + ex_path)
        self.controller_class = template_src.Controller()

        # Set up variable which measures how long it took since last command
        self.last_ms = None

        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
        rospy.loginfo("\n\n\nREADY FOR EXERCISE " + exercise_txt + "\n\n\n")

    # Methods to extract a number from a string and remove a number from a string
    def get_num(self, x):
        return int(''.join(ele for ele in x if ele.isdigit()))

    def get_txt(self, x):
        return ''.join([i for i in x if not i.isdigit()])

    def cbPose(self, lane_pose_msg):

        lane_reading = lane_pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - lane_reading.header.stamp

        # delay from taking the image until now in seconds
        t_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        # Calculate time since last command
        currentMillis = int(round(time.time() * 1000))
        if self.last_ms is not None:
            dt_last = (currentMillis - self.last_ms) / 1000.0
        else:
            dt_last = 0 # None before, let's make 0 such that it is way simpler for students

        # Obtaining parameters to give to controller_class
        d_est = lane_pose_msg.d
        phi_est = lane_pose_msg.phi
        d_ref = 0
        phi_ref = 0
        v_ref = 0.38

        # Obtain new v and omega
        v_out, omega_out = self.controller_class.getControlOutput(d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last)

        # Print out infos
        rospy.loginfo("Omega: " + str(omega_out) + "    V: " + str(v_out) + "    Err: " + str(d_est - d_ref))

        # Create message and publish
        car_control_msg = Twist2DStamped()
        car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = v_out
        car_control_msg.omega = omega_out
        self.publishCmd(car_control_msg)

        # Update last timestamp
        self.last_ms = currentMillis


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
