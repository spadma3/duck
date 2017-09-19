#!/usr/bin/python

# Use a DC Motor controller on the Adafruit DC Stepper board to create a patrol-like red/blue flashing light effect
#
# date:    09/19/2017
#
# authors: Andrea F. Daniele <afdaniele@ttic.edu>

import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor
import signal
import sys
import time

class PatrolLights:
    LIGHTS_PWM = 60        # Minimum voltage signal required to turn on the lights

    def __init__(self, verbose=False, debug=False):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.lightsControl = self.motorhat.getMotor(1)
        self.verbose = verbose or debug
        self.debug = debug

        self.lights_on = False
        self.lights_current_mode_id = -1
        self.lights_modes = [ Adafruit_MotorHAT.FORWARD, Adafruit_MotorHAT.BACKWARD ]
        self.lights_timer = rospy.Timer(rospy.Duration.from_sec(0.32), self.blink)

        # Subscriptions
        self.sub_patrol = rospy.Subscriber("patrol", BoolStamped, self.updateLights, queue_size=1)

    def updateLights(self, patrol_msg):
        if self.lights_current_mode_id < -1: return
        self.lights_on = patrol_msg.data


    def blink(self, event):
        if not self.lights_on:
            if self.lights_current_mode_id > -1:
                self.lights_current_mode_id = -2
                self.off()
                time.sleep(0.4)
                self.lights_current_mode_id = -1
            return
        # turn ON the lights / blink
        self.lightsControl.setSpeed( PatrolLights.LIGHTS_PWM )
        self.lights_current_mode_id = (self.lights_current_mode_id + 1) % 2
        self.lightsControl.run( self.lights_modes[ self.lights_current_mode_id ] )

    def off(self):
        self.lights_on = False
        self.lightsControl.setSpeed( 0 )
        self.lightsControl.run( Adafruit_MotorHAT.RELEASE )
        self.lights_current_mode_id = -1


if __name__ == "__main__":
    rospy.init_node("patrol_bot", anonymous=False)
    patrol_bot = PatrolLights()
    def signal_handler(signal, frame):
        patrol_bot.off()
        time.sleep(1)
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()
