import math
import numpy as np


class Controller():

    def __init__(self):

        # Gains for controller
        self.k = 6
        self.k_I = 6

        # Assumed saturation of motors [rad/s] (this is what we think to know)
        self.u_sat = 5.5

        # Feedback gain for anti-windup
        self.k_t = 12

        # Variable for integral
        self.integral = 0

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

        # Calculate the output y
        y = 6 * (d_est - d_ref) + 1 * (phi_est-phi_ref)

        # PI-Controller
        omega = -self.k * y + self.integral

        # Integrate the error while considering the anti-windup
        self.integral = self.integral + dt_last * ( self.k_I * (-y) + self.k_t*( self.sat(omega) - omega ) )

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)

    # Defining the saturation function of the motors (this is what we assume)
    def sat(self, u):
        if u > self.u_sat:
            return self.u_sat
        if u < -self.u_sat:
            return -self.u_sat
        return u
