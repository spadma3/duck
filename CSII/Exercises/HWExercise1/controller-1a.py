import math

class Controller():

    def __init__(self):

        # Variables needed for controller (None for P-controller)
          # self.d_err = 0
          # self.d_integral = 0

        # Gains for controller
        self.k_d = -10.3
        self.k_theta = -5.15


    # Inputs:   d_est   Estimation of distance from lane center (positve when
    #                   offset to the left of driving direction) [m]
    #           phi_est Estimation of angle of bot (positive when angle to the
    #                   left of driving direction) [rad]
    #           d_ref   Reference of d (for lane following, d_ref = 0) [m]
    #           v_ref   Reference of velocity [m/s]
    #           t_delay Delay it took from taking image up to now [s]
    #           dt_last Time it took from last processing to current [s]

    # Output:   v_out       velocity of Duckiebot [gain, element of [0,1]]
    #           omega_out   angular velocity of Duckiebot []

    def getControlOutput(self, d_est, phi_est, d_ref, phi_ref, v_ref, t_delay, dt_last):

        # Calculating the errors of d and phi
        d_err = d_est - d_ref
        phi_err = phi_est - phi_ref

        # Native P-Controller for error of d
        omega =  self.k_d * d_err

        # Native P-Controller for error of phi
        omega = omega + self.k_theta * phi_err

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)



    # def sign(self, x):
    #     if x == 0:
    #         return 0
    #     if x > 0:
    #         return 1
    #     else:
    #         return -1
