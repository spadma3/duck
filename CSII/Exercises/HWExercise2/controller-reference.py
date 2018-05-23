import math
import numpy as np
import control

class Controller():

    def __init__(self):

        #Define cost matrices here:
        #defnfine matrices with: self.A = numpy.array([[a11, a12], [a21, a22]])
        self.d_int = 0.
        self.q = np.array([[80., 0, 0], [0, 5., 0], [0, 0, 2]])
        self.r = np.array([0.02])
        self.gain = 0.5

        #TODO

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
        #Exception for dt_last=0
        if (dt_last==0) : dt_last=0.0001
        #Update state
        v_ref = self.gain*v_ref
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
        omega_out = -g*x*self.gain
        #Update integral term
        self.d_int = self.d_int + d_est * dt_last
        return (v_out, omega_out)