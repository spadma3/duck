import math

class Controller():

    def __init__(self):

        # Gains for controller
        self.k_P = 3.5
        self.k_I = 1.2

        # Variable for integral
        self.C_I = 0

        # Specify the time delay. k_d = 1 means that there is a time delay
        # t_delay = t_delay(0) + k_d * T where T = 70ms
        self.k_d = 0

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
        ref =   (6 * d_ref + 1 * phi_ref)
        y =     (6 * d_est + 1 * phi_est)
        err = ref - y

        # PI-Controller
        C_P = self.k_P * err
        omega = C_P + self.C_I

        # Calculate the new value of the integral
        self.C_I = self.C_I + dt_last * self.k_I * err

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)
