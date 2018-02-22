import math

class Controller():

    def __init__(self):

        # Gains for controller
        self.k = 9.5
        self.k_I = 3

        # Variable for integral
        self.integral = 0

        # Specify the sampling time. k_s = 2 means that the time between two
        # executions gets doubled (for k_s= 1: T = 70ms)
        self.k_s = 1

    # Inputs:   d_est   Estimation of distance from lane center (positve when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [rad]
    #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           t_delay Delay it took from taking image up to now [s]
    #           dt_last Time it took from last processing to current [s]

    # Output:   v_out       velocity of Duckiebot [gain, element of [0,1]]
    #           omega_out   angular velocity of Duckiebot [rad/s]

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):

        # Calculate the output y
        y = 6 * (d_est - d_ref) + 1 * (phi_est-phi_ref)

        # Integrate y
        self.integral = self.integral + y * dt_last

        # PI-Controller
        omega = -self.k * y - self.k_I * self.integral

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)
