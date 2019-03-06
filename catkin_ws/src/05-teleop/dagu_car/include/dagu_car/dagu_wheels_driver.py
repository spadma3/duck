#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#          Shih-Yuan Liu <syliu@mit.edu>

from Adafruit_MotorHAT import Adafruit_MotorHAT
import RPi.GPIO as GPIO                                                         #import for the servo control, RFMH_2019_03_05
from math import fabs, floor
from time import sleep                                                          #import for the servo control, RFMH_2019_03_05

class DaguWheelsDriver:
    #LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor              #the left motor is not used anymore --> changed to servo!, RFMH_2019_02_26
    #LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor
    DC_MOTOR_MIN_PWM = 60           # Minimum speed for right motor             #change the name from "RIGHT_MOTOR_MIN_PWM" to "DC_MOTOR_MIN_PWM", RFMH_2019_02_26
    DC_MOTOR_MAX_PWM = 255          # Maximum speed for right motor             #change the name from "RIGHT_MOTOR_MAX_PWM" to "DC_MOTOR_MAX_PWM", RFMH_2019_02_26
    # AXEL_TO_RADIUS_RATIO = 1.0    # The axel length and turning radius ratio
    SPEED_TOLERANCE = 1.e-2         # speed tolerance level

    def __init__(self, verbose=False, debug=False, left_flip=False, dcmotor_flip=False):
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        #self.leftMotor = self.motorhat.getMotor(2)                             #commented, RFMH_2019_02_26
        self.DcMotor = self.motorhat.getMotor(1)                                #change the name from "rightMotor" to "DcMotor", RFMH_2019_02_26
        self.verbose = verbose or debug
        self.debug = debug

        GPIO.setmode(GPIO.BOARD)                                                #Declaration of all the pins by setting the naming mode, RFMH_2019_03_05
        GPIO.setup(32, GPIO.OUT)                                                #Set the output to GPIO 12 (pin 32), RFMH_2019_03_05
        pwm = GPIO.PWM(32, 50)                                                  #setup the PWM frequency, RFMH_2019_03_05
        pwm.start(0)                                                            #initialization, RFMH_2019_03_05

        # self.left_sgn = 1.0                                                   #commented, RFMH_2019_02_26
        # if left_flip:
        #     self.left_sgn = -1.0

        self.dcmotor_sgn = 1.0                                                  #line 34-36: changed everywhere "right_sgn" to "dcmotor_sgn", RFMH_2019_02_26
        if dcmotor_flip:
            self.dcmotor_sgn = -1.0

        #self.leftSpeed = 0.0                                                   #commented, RFMH_2019_02_26
        self.dcmotorSpeed = 0.0                                                 #change "rightSpeed" to "dcmotorSpeed", RFMH_2019_02_26
        self.updatePWM()

    def SetAngle(self, angle):                                                        #function to set the angle on the servo, RFMH_2019_03_05
    	duty = angle / 18.0 + 2                                                 #TODO: check if this duty cycle from 2-12% does still hold for the final versino of the servo, RFMH_2019_03_05
    	GPIO.output(32, True)                                                   #Set the GPIO12 to an active output, RFMH_2019_03_05
    	pwm.ChangeDutyCycle(duty)                                               #Set the actual PWM-signal, RFMH_2019_03_05
    	sleep(0.5)                                                              #wait for the servo to set the angle hardware-wise, RFMH_2019_03_05 (maybe change to 1sec)
    	GPIO.output(32, False)
    	pwm.ChangeDutyCycle(0)                                                  #implmenting this preserves the servo from jittering, RFMH_2019_03_05

    def PWMvalue(self, v, minPWM, maxPWM):
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
        return min(pwm, maxPWM)

    def updatePWM(self):
        #vl = self.leftSpeed*self.left_sgn                                      #commented, RFMH_2019_02_26
        v_dc = self.dcmotorSpeed*self.dcmotor_sgn                               #changed "vr" to "v_dc", "rightSpeed" to "dcmotorSpeed" and "right_sgn" to dcmotor_sgn", RFMH_2019_02_26

        #pwml = self.PWMvalue(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)         #commented because out of use, RFMH_2019_02_26
        pwm_dc = self.PWMvalue(v_dc, self.DC_MOTOR_MIN_PWM, self.DC_MOTOR_MAX_PWM)    #changed "pwmr" to "pwm_dc" and "vr" to "v_dc" and adjusted both orange constants to "DC_MOTOR_MIN_PWM" AND "DC_MOTOR_MAX_PWM", RFMH_2019_02_26

        if self.debug: #where the duck does the "u" come from?!?, RFMH_2019_02_26
            print "v = %5.3f, u = %5.3f, v_dc = %5.3f, pwm_dc = %3d" % (v, u, v_dc, pwm_dc) #deleted "vl" and "pwml" and adjust "vr" to "v_dc" to "pwm_dc"

        # if fabs(vl) < self.SPEED_TOLERANCE:                                   #commented out the left motor as only one will be used RFMH_2019_02_28
        #     leftMotorMode = Adafruit_MotorHAT.RELEASE
        #     pwml = 0
        # elif vl > 0:
        #     leftMotorMode = Adafruit_MotorHAT.FORWARD
        # elif vl < 0:
        #     leftMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(v_dc) < self.SPEED_TOLERANCE:                                   #changed v_r to v_dc in if loop , RFMH_2019_02_28
            DcMotorMode = Adafruit_MotorHAT.RELEASE
            pwmr = 0
        elif v_dc > 0:
            DcMotorMode = Adafruit_MotorHAT.FORWARD
        elif v_dc < 0:
            DcMotorMode = Adafruit_MotorHAT.BACKWARD

        # self.leftMotor.setSpeed(pwml)                                         #commented out the left motor as only one will be used RFMH_2019_02_28
        # self.leftMotor.run(leftMotorMode)
        self.DcMotor.setSpeed(pwm_dc)                                           # changed rightMotor to DcMotor and pwmr to pwm_dc , RFMH_2019_02_28
        self.DcMotor.run(DcMotorMode)

    def setWheelsSpeed(self, dc_motor_speed):                                   #POSSIBLE BUG: method had 3 arguments (self, left, right) , now changed to only 2 (self, dc_motor_speed)
        #self.leftSpeed = left                                                  #commented out because we only use one motor , RFMH_2019_02_28
        self.dcmotorSpeed = dc_motor_speed                                      #changed rightSpeed to dcmotorSpeed and right to
        self.updatePWM()

    def __del__(self):
        #self.leftMotor.run(Adafruit_MotorHAT.RELEASE)                          #only one needed
        self.DcMotor.run(Adafruit_MotorHAT.RELEASE)                             #changed rightMotor to DcMotor , RFMH_2019_02_28
        del self.motorhat
        pwm.stop()                                                              #shutting down procedure for the gain_servo, RFMH_2019_03_05
        GPIO.cleanup()

# Simple example to test motors
if __name__ == '__main__':
    from time import sleep

    N = 10
    delay = 100. / 1000.

    dagu = DAGU_Differential_Drive()

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    # turn right
    dagu.setSteerAngle(-1.0)
    # accelerate backward
    for i in range(N):
        dagu.setSpeed(-(1.0 + i) / N)
        sleep(delay)
    # decelerate backward
    for i in range(N):
        dagu.setSpeed(-(-1.0 - i + N) / N)
        sleep(delay)

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    del dagu
