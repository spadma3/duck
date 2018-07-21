import math
import numpy
import control

class Controller():

    def __init__(self):
        # Gains for controller
        self.q = numpy.array([[70., 0], [0, 3.]])
        self.r = numpy.array([0.007])

    # Inputs:   d_est   Estimation of distance from lane center (positve when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [rad]
    #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           t_delay Delay it took from taking image up to now [s]
    #           dt_last Time it took from last processing to current [s]
    # Output:   v_out       velocity of Duckiebot [m/s]
    #           omega_out   angular velocity of Duckiebot [rad/s]
    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):
        #Exception handler for dt_last==0
        if(dt_last==0):
            dt_last=0.0001
        x = numpy.array([[d_est],[phi_est]])
        #Discrete State Space Matrices
        a = numpy.array([[1., v_ref*dt_last], [0, 1.]])
        b = numpy.array([[v_ref*dt_last*dt_last/2], [dt_last]])
        #Solve Discrete Algebraic Riccati Equation
        (x_ric, l, g) = control.dare(a, b, self.q, self.r)
        v_out = v_ref
        omega_out = -numpy.dot(g,x)
        return (v_out, omega_out)
