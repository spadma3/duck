import math
import numpy as np
import rospy

class Controller():

    def __init__(self):

        # Gains for controller, obtained by using lqr in MATLAB
        self.K = [[0, 5, -1.81], [2, 0, 0]]


        # Variables
        self.r0 = 0.23
        self.v0 = 0
        self.vmax = 0.3
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
        d = -rho * np.sin(theta + psi)
        alpha = -psi
        r = rho * np.cos(theta + psi)

        rospy.loginfo("d: " + str(d) + "    alpha: " + str(alpha) + "    r: " + str(r))
        # Calculate current state
        delta_x = [r - self.r0, d, alpha]

        # Calculate new input
        u_equi = [0, self.v0]
        delta_u = np.dot(self.K,delta_x)
        u = np.add(delta_u, u_equi)

        v_ref = u[1]
        omega = u[0]

        # v_ref > 0
        v_ref = v_ref if v_ref > 0 else 0
        v_ref = v_ref if v_ref < self.vmax else self.vmax

        omega = omega if omega < 1.5 else 1.5
        omega = omega if omega > -1.5 else -1.5
        # Declaring return values
        omega_out = omega
        v_out = v_ref
        return (v_out, omega_out)
