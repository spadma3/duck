#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState, BoolStamped
import os, imp, time



################# NOTE TO ALL TEACHING ASSISTANTS!!! ###################
# IF you need to customize the behavior of a part exercise outside     #
# the exercise itself like for example in HWExercise 1-3 (saturation)  #
# or HWExercise 1-4 (different sampling rate) then please edit this in #
# the marked regions "CUSTOMIZATION". This will ensure a "clear"       #
# structure. Just orient yourself at the examples of HWExercise 1.     #
########################################################################


class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("~lane_pose", LanePose, self.cbPose, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~switch", BoolStamped, self.cbMode, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # Load files for HW-Exercises
        exercise_txt = os.environ['CSII_EXERCISE']
        exercise = exercise_txt.split("-")
        self.exercise = exercise

        duckietown_root = os.environ['DUCKIETOWN_ROOT']

        ex_path = "/CSII/Exercises/HWExercise" + str(exercise[0]) + "/controller-" + str(exercise[1]) + ".py"
        template_src = imp.load_source('module.name', duckietown_root + ex_path)
        self.controller_class = template_src.Controller()

        # HACK: Add listener for FSM machine in order to avoid integrating if not in autopilot mode
        veh_name = os.environ['VEHICLE_NAME']
        self.sub_fsm_mode = rospy.Subscriber("/" + str(veh_name) + "/fsm_node/mode", FSMState, self.cbMode, queue_size=1)

        # Set up variable which measures how long it took since last command
        self.last_ms = None

        # Set up operating variable
        self.operating = False

        # Setup variable for different sampling rate
        self.z_samp = 0

        # Setup array for time delay
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 5:
            k_d = self.controller_class.k_d
            if k_d > 0:
                self.arr_delay = [[0, 0, 0, 0, 0, 0, 0]] # d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last
                for i in range(1, k_d):
                    self.arr_delay.append([0, 0, 0, 0, 0, 0, 0])

        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))
        rospy.loginfo("\n\n\n\n\nREADY FOR EXERCISE " + exercise_txt + "\n\n\n\n\n")


    def cbPose(self, lane_pose_msg):
        self.z_samp += 1

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

        # Return if not in autopilot
        if not self.operating:
            return

        # Obtaining parameters to give to controller_class
        d_est = lane_pose_msg.d
        phi_est = lane_pose_msg.phi
        d_ref = 0
        phi_ref = 0
        v_ref = 0.22



        ########## SUBEXERCISE CUSTOMIZATION BEFORE CONTROLLER ##########

        # SAMPLING RATE ADJUSTMENT IN EXERCISE 1-4
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 4:
            k_s = self.controller_class.k_s
            if k_s != 0:
                if self.z_samp % k_s != 0:
                    return
            else:
                dt_last *= k_s # HACK: supposing constant sampling rate. Approx. true for our purposes.
                self.z_samp = 0

        # TIME DELAY IN EXERCISE 1-5
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 5:
            k_d = self.controller_class.k_d
            if k_d > 0:
                self.arr_delay.append([d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last])
                d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last = self.arr_delay.pop(0)
                t_delay += k_d * dt_last # HACK: same as in ex 1-4

        ########## END SUBEXERCISE CUSTOMIZATION BEFORE CONTROLLER ##########

        # Obtain new v and omega
        v_out, omega_out = self.controller_class.getControlOutput(d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last)


        ########## SUBEXERCISE CUSTOMIZATION AFTER CONTROLLER ##########

        # SATURATION IN EXERCISE 1-3
        if int(self.exercise[0]) == 1 and int(self.exercise[1]) == 3:
            omega_max = 5.5
            if omega_out > omega_max:
                omega_out = omega_max
            if omega_out < -omega_max:
                omega_out = -omega_max



        ########## END SUBEXERCISE CUSTOMIZATION AFTER CONTROLLER ##########


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


    # FSM
    def cbMode(self,fsm_state_msg):
        self.operating = fsm_state_msg.state == "LANE_FOLLOWING"



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
