import math

class Controller():

    def __init__(self):

        # Gains for controller
        self.k = 3.5


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

    def getControlOutput(self, rho, theta, psi, t_delay, dt_last):

        omega = 0
        v_ref = 0
        if rho > 0.25:
            v_ref = 0.2

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)
