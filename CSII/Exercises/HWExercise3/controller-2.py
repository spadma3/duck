import math
import numpy as np
import rospy
import control

class Controller():

    def __init__(self):

        # Gains for controller, obtained by using lqr in MATLAB
        self.d_int = 0.
        self.q = np.array([[80., 0, 0], [0, 5., 0], [0, 0, 2]])
        self.r = np.array([0.04])

        # Variables
        self.r0 = 0.13
        self.v0 = 0.2
        #self.vmax = 0.3
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
        rospy.loginfo(rho)
        rospy.loginfo(psi)
        rospy.loginfo(theta)
        # Calculate new coordinates
        # d = -rho * np.sin(theta + psi)
        # alpha = -psi
        r = rho * np.cos(theta + psi)
        d_est = rho * np.sin(theta + psi)
        phi_est = -psi
        v_ref = self.v0 + 1.8*(r-self.r0)
        #Exception for dt_last=0
        if (dt_last==0) : dt_last=0.0001
        #Update state
        x = np.array([[d_est],[phi_est],[self.d_int]])
        #Adapt State Space to current time step
        a = np.array([[1., v_ref*dt_last, 0], [0, 1., 0], [-dt_last, -0.5*v_ref*dt_last*dt_last, 1]])
        b = np.array([[0.5*v_ref*dt_last*dt_last], [dt_last], [-v_ref*dt_last*dt_last*dt_last/6]])
        #Solve ricatti equation
        (x_ric, l, g) = control.dare(a, b, self.q, self.r)
        #feed trough velocity
        v_out = v_ref
        #Change sign on on K_I (because Ricatti equation returns [K_d, K_phi, -K_I])
        g[[0],[2]] = -g[[0],[2]]
        #Calculate new input
        omega_out = -g*x
        #Update integral term
        self.d_int = self.d_int + d_est * dt_last
        return (v_out, omega_out)

        # rospy.loginfo("d: " + str(d) + "    alpha: " + str(alpha) + "    r: " + str(r))
        # # Calculate current state
        # delta_x = [r - self.r0, d, alpha]
        #
        # # Calculate new input
        # u_equi = [0, self.v0]
        # delta_u = np.dot(self.K,delta_x)
        # u = np.add(delta_u, u_equi)
        #
        # v_ref = u[1]
        # omega = u[0]
        #
        # # v_ref > 0
        # v_ref = v_ref if v_ref > 0 else 0
        # v_ref = v_ref if v_ref < self.vmax else self.vmax
        #
        # omega = omega if omega < 1.5 else 1.5
        # omega = omega if omega > -1.5 else -1.5
        # # Declaring return values
        # omega_out = omega
        # v_out = v_ref
        # return (v_out, omega_out)
