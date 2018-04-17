import math
import numpy as np

class Controller():

    def __init__(self):

        # Gains for controller, obtained by using lqr in MATLAB
        self.K = [[0, 7.07, 1.81], [-2, 0, 0]]


        # Variables
        self.r0 = 0.3
        self.v0 = 0.22

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

        # Calculate new coordinates
        d = -rho * sin(theta + psi)
        alpha = -psi
        r = rho * cos(theta + psi)

        # Calculate current state
        delta_x = [r - self.r0, d, alpha]

        # Calculate new input
        u_equi = [0, self.v0]
        delta_u = np.dot(K,delta_x)
        u = np.add(delta_u, u_equi)

        v_ref = u[1]
        omega = u[0]

        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)
