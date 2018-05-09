#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import Twist2DStamped
import time
import pygame

class SteeringWheelNode(object):
    def __init__(self):
        self.node_name = "Steering Wheel"

        ## params
        self.k_angle = self.setupParam("~k_angle", 0.5) # Relation between angle of steering wheel and front wheels [1]
        self.acc_max = self.setupParam("~acc_max", 1) # Maximal acceleration (full throttle) [m/s^2]
        self.dec_max = self.setupParam("~dec_max", 2) # Maximal deceleration (breaking -> negative) [m/s^2]
        self.dec_nom = self.setupParam("~dec_nom", 0.5) # Nominal deceleration if throttle == 0 [m/s^2]
        self.v_max   = self.setupParam("~v_max", 1) # Maximal velocity possible [m/s]
        self.v_r_max   = self.setupParam("~v_r_max", 0.15) # Mximal velocity possible in reverse [m/s]
        self.angle_max = self.setupParam("~angle_max", np.pi/3) # Maximal possible angle of front wheels in both directions [rad]
        self.wheel_base = self.setupParam("~wheel_base", 3.125) # Distance between front and back wheels [m]
        self.accel_fun = self.setupParam("~accel_fun(v,v_max, acc_max, throttle)", 1)
        ## publishers and subscribers
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

        self.v = 0 # Current longitudinal velocity
        self.omega = 0 # Current angular velocity
        self.last_ms = None # For integrating acceleration
        self.gear = "PARK" # DRIVE, REVERSE, NEUTRAL, PARK

        # Prepate pygame listener
        pygame.init()
        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
        self.joy = joysticks[0]
        self.joy.init()
        rospy.loginfo(self.joy.get_numaxes())

        # Start main loop
        self.mainLoop = rospy.Timer(rospy.Duration.from_sec(0.05), self.mainLoop)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    def mainLoop(self, event):
        pygame.event.pump()


        steering_wheel_angle = -self.joy.get_axis(0)
        throttle_amp = (1-self.joy.get_axis(2))/0.5
        break_amp = (1-self.joy.get_axis(3))/0.5

        w_left = self.joy.get_button(5)
        w_right = self.joy.get_button(4)

        rospy.loginfo("Gear: " + self.gear + "    Angle: " + str(steering_wheel_angle) + "    Throttle: " + str(throttle_amp) +  "    Break: " + str(break_amp) + "    Omega: " + str(self.omega) + "    V: " + str(self.v))

        # Apply driving actions to Duckiebot
        self.cbSteeringWheelActions(steering_wheel_angle, throttle_amp, break_amp)

        if w_right and w_left:
            if np.abs(self.v) < 0.01:
                self.gear = "PARK"
            else:
                self.gear = "NEUTRAL"
        else:
            if w_left:
                self.gear = "REVERSE"
            if w_right:
                self.gear = "DRIVE"

    def updateParams(self,event):
        self.k_angle = rospy.get_param("~k_angle") # Relation between angle of steering wheel and front wheels [1]
        self.acc_max = rospy.get_param("~acc_max") # Maximal acceleration (full throttle) [m/s^2]
        self.dec_max = rospy.get_param("~dec_max") # Maximal deceleration (breaking -> negative) [m/s^2]
        self.dec_nom = rospy.get_param("~dec_nom") # Nominal deceleration if throttle == 0 [m/s^2]
        self.v_max   = rospy.get_param("~v_max") # Maximal velocity possible [m/s]
        self.v_r_max   = rospy.get_param("~v_r_max")
        self.angle_max = rospy.get_param("~angle_max")
        self.wheel_base = rospy.get_param("~wheel_base")
        self.accel_fun = rospy.get_param("~accel_fun(v,v_max, acc_max, throttle)")


    def getAcceleration(self, v, v_max, acc_max, throttle):
        acc = eval(self.accel_fun)
        return acc

    def cbSteeringWheelActions(self, steering_wheel_angle, throttle_amp, break_amp):

        # Calculate front wheels angle and cut off if limit exceeded
        front_wheels_angle = steering_wheel_angle * self.k_angle
        front_wheels_angle = np.clip(front_wheels_angle, -self.angle_max, self.angle_max)

        # Calculate new velocity
        currentMillis = self.getCurrentMillis()
        if self.last_ms is not None:
            delta_t = (currentMillis - self.last_ms)/1000.0
            if self.gear == "DRIVE":
                self.v = self.v + delta_t * self.getAcceleration(self.v, self.v_max, self.acc_max, throttle_amp)
                self.v = self.v - delta_t * break_amp * self.dec_max
                self.v = self.v - delta_t * self.dec_nom
                self.v = np.clip(self.v, 0, self.v_max)
            if self.gear == "REVERSE":
                self.v = self.v - delta_t * self.getAcceleration(self.v, self.v_max, self.acc_max, throttle_amp)
                self.v = self.v + delta_t * break_amp * self.dec_max
                self.v = self.v + delta_t * self.dec_nom
                self.v = np.clip(self.v, -self.v_r_max, 0)
            if self.gear == "NEUTRAL":
                if self.v >= 0:
                    self.v = self.v - delta_t * break_amp * self.dec_max
                    self.v = self.v - delta_t * self.dec_nom
                    self.v = np.clip(self.v, 0, self.v_max)
                else:
                    self.v = self.v + delta_t * break_amp * self.dec_max
                    self.v = self.v + delta_t * self.dec_nom
                    self.v = np.clip(self.v, -self.v_r_max, 0)
            if self.gear == "PARK":
                self.v = 0

        self.last_ms = currentMillis

        # Calculate longitudinal velocity
        v_l = self.v * np.cos(front_wheels_angle)

        # Calculate angular velocity
        self.omega = self.v / self.wheel_base * (np.tan(front_wheels_angle))

        # Publish update
        car_control_msg = Twist2DStamped()
        car_control_msg.v = self.v
        car_control_msg.omega = self.omega
        self.pub_car_cmd.publish(car_control_msg)

    def onShutdown(self):
        rospy.loginfo("[SteeringWheelNode] Shutdown.")
        self.mainLoop.shutdown()
    def getCurrentMillis(self):
        return int(round(time.time() * 1000))

if __name__ == '__main__':
    rospy.init_node('steering_wheel_node',anonymous=False)
    steering_wheel_node = SteeringWheelNode()
    rospy.on_shutdown(steering_wheel_node.onShutdown)
    rospy.spin()
