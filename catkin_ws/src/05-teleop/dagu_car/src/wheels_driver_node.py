#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from dagu_car.dagu_wheels_driver import DaguWheelsDriver
import numpy as np

class WheelsDriverNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        self.estop=False

        # Parameters for maximal turning radius
        self.use_rad_lim        =   self.setupParam("~use_rad_lim", False)
        #self.min_rad            =   self.setupParam("~min_rad", 0.13)          #not needed as we defined the extreme positions of the steering angle , RFMH_2019_02_28
        #self.angle_lim_left     = self.setupParam("~angle_lim_left", 0.611)    #TODO outcomment if we need to set the parameters , RFMH_2019_02_28
        #self.angle_lim_right    = self.setupParam("~angle_lim_right", -0.611)  #TODO outcomment if we need to set the parameters , RFMH_2019_02_28
        self.wheel_distance     =   self.setupParam("~wheel_distance", 0.092)   #wheel_distance default value changed to 0.092, RFMH_2019_02_26


        # Setup publishers
        self.DCdriver = DaguWheelsDriver()                                      #rename driver to DCdriver
        self.ServoDriver = DaguWheelsDriver()                                   #creaste new object ServoDriver
        #add publisher for wheels command wih execution time
        self.msg_wheels_cmd = WheelsCmdStamped()
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.control_constant = 1.0
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        self.sub_e_stop = rospy.Subscriber("~emergency_stop", BoolStamped, self.cbEStop, queue_size=1)
        #self.sub_rad_lim = rospy.Subscriber("~radius_limit", BoolStamped, self.cbRadLimit, queue_size=1)           #not needed TODO: get rid of publisher as well, not needed

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def updateParams(self,event):
        self.use_rad_lim = rospy.get_param("~use_rad_lim")
        #self.min_rad = rospy.get_param("~min_rad")                             #min_rad not used, see line 15 , RFMH_2019_02_28
        self.wheel_distance = rospy.get_param("~wheel_distance")
        #self.angle_lim_left = self.get_param("~angle_lim_left")                #TODO outcomment if we need to set the parameters , RFMH_2019_02_28
        #self.angle_lim_right = self.get_param("~angle_lim_right")              #TODO outcomment if we need to set the parameters , RFMH_2019_02_28

    def cbWheelsCmd(self,msg):
        if self.estop:
            self.DCdriver.setWheelsSpeed(dc_motor_speed=0.0)                    #replace the two arguments "left" and "right" by only one called "dc_motor_speed" and rename driver to DCdriver
            self.ServoDriver.SetAngle(angle=0.0)                                #set angle of servo to zero if stopped
            return


        self.DCdriver.setWheelsSpeed(dc_motor_speed=msg.vel_wheel)              #deleted argument for vel_left, changed "right" to "dc_motor_speed" and "vel_right" to "vel_wheel", RFMH_2019_02_26
        self.ServoDriver.SetAngle(angle = msg.gamma)                            #set the steering angle on the servo, RFMH_2019_03_05
        # Put the wheel commands in a message and publish
        self.msg_wheels_cmd.header = msg.header
        # Record the time the command was given to the wheels_driver
        self.msg_wheels_cmd.header.stamp = rospy.get_rostime()
        self.msg_wheels_cmd.gamma = msg.gamma                                   #TODO: check whether gamma is published correctly with this implementation
        self.msg_wheels_cmd.vel_wheel = msg.vel_wheel                           #"vel_right" changed to "vel_wheel", RFMH_2019_02_26
        self.pub_wheels_cmd.publish(self.msg_wheels_cmd)


    # def checkAndAdjustRadius(self, msg):                                      #replaced by AdjustAngle()
    #     didAdjustment = False
    #     # if both motor cmds do not have the same sign, we're demanding for an on-point turn (not allowed)
    #     if (np.sign(msg.vel_left) != np.sign(msg.vel_right)):
    #
    #         # Simply set the smaller velocity to zero
    #         if (abs(msg.vel_left) < abs(msg.vel_right)):
    #             msg.vel_left = 0.0
    #         else:
    #             msg.vel_right = 0.0
    #
    #         didAdjustment = True
    #
    #     # set v1, v2 from msg velocities such that v2 > v1
    #     if (abs(msg.vel_right) > abs(msg.vel_left)):
    #         v1 = msg.vel_left
    #         v2 = msg.vel_right
    #     else:
    #         v1 = msg.vel_right
    #         v2 = msg.vel_left
    #
    #     # Check if a smaller radius than allowed is demanded
    #     if (v1 == 0 or abs(v2 / v1) > (self.min_rad + self.wheel_distance/2.0)/(self.min_rad - self.wheel_distance/2.0)):
    #
    #         # adjust velocities evenly such that condition is fulfilled
    #         delta_v = (v2-v1)/2 - self.wheel_distance/(4*self.min_rad)*(v1+v2)
    #         v1 += delta_v
    #         v2 -= delta_v
    #         didAdjustment = True
    #
    #     # set msg velocities from v1, v2 with the same mapping as when we set v1, v2
    #     if (abs(msg.vel_right) > abs(msg.vel_left)):
    #         msg.vel_left = v1
    #         msg.vel_right = v2
    #     else:
    #         msg.vel_left = v2
    #         msg.vel_right = v1
    #
    #     return didAdjustment


    def cbEStop(self,msg):
        self.estop=not self.estop
        if self.estop:
            rospy.loginfo("[%s] Emergency Stop Activated")
        else:
            rospy.loginfo("[%s] Emergency Stop Released")

    def on_shutdown(self):
        self.DCdriver.setWheelsSpeed(dc_motor_speed=0.0)                        #changed (left=0.0, right=0.0) to (dc_motor_speed=0.0)
        self.ServoDriver.SetAngle(angle=0.0)                                    #on shutdown place steering to 0 degrees
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverNode()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
