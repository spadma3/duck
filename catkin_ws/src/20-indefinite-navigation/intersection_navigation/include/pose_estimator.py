#!/usr/bin/env python
import numpy as np
from collections import deque

class VehicleCommands(object):
    '''class for storing vehicle commands'''

    def __init__(self,v,omega,time):
        self.v = v
        self.omega = omega
        self.time = time


class PoseEstimator(object):
    '''class for estimating pose of Duckiebot at intersection'''

    def __init__(self):
        # estimator state
        self.state_est = np.zeros(shape=3, dtype=float) # state = (x,y,theta)
        self.cov_est = np.zeros(shape=(3,3), dtype=float)
        self.time_est = 0.0

        # vehicle command queue
        self.cmd_queue = deque([VehicleCommands(0.0, 0.0, 0)])

        # estimator parameters
        self.cov_proc = np.diag([0.1,0.1]) # process noise is assumed to be on inputs

    def Reset(self, x_init, y_init, theta_init, time_init):
        '''reset state estimate'''
        self.state_est[0] = x_init
        self.state_est[1] = y_init
        self.state_est[2] = theta_init

        self.time_est = time_init

        self.cov_est = np.diag([1.0, 1.0, 1.0])

    def PredictState(self, time_pred, predict_cov=False):
        '''predict estimate forward until time_pred'''
        state_est = self.state_est[:, :]
        cov_est = self.cov_est[:,:]
        time_est = self.time_est

        # integrate forward with vehicle commands
        idx_cmd = 0
        num_cmd = len(self.cmd_queue)
        while time_est < time_pred:
            # find current command
            if idx_cmd + 1 < num_cmd:
                dt = min(self.cmd_queue[idx_cmd+1].time, time_pred) - time_est # careful, this could eventually cause problems if running long
            else:
                dt = time_pred - time_est

            # predict covariance
            if predict_cov:
                A = np.array([[0.0, 0.0, -self.cmd_queue[idx_cmd].v*np.sin(state_est[2])],
                             [0.0, 0.0, self.cmd_queue[idx_cmd].v*np.cos(state_est[2])],
                            [0.0, 0.0, 0.0]])
                L = np.array([[np.cos(state_est[2]), 0.0],
                              [np.sin(state_est[2]), 0.0],
                              [0.0, 1.0]])
                cov_dot = np.dot(A,cov_est) + np.dot(cov_est, A.T) + np.dot(L, np.dot(self.cov_proc, L.T))
                cov_est = cov_est + cov_dot*dt

            # predict state
            if np.abs(self.cmd_queue[idx_cmd].omega) > 1e-6:
                radius = self.cmd_queue[idx_cmd].v / self.cmd_queue[idx_cmd].omega

                state_est[0] = (state_est[0] - radius*np.sin(state_est[2])) + radius*np.sin(state_est[2] + self.cmd_queue[idx_cmd].omega * dt)
                state_est[1] = (state_est[1] + radius*np.cos(state_est[2])) - radius*np.cos(state_est[2] + self.cmd_queue[idx_cmd].omega * dt)
            else:
                state_est[0] = state_est[0] + self.cmd_queue[idx_cmd].v*np.cos(state_est[2])*dt
                state_est[1] = state_est[1] + self.cmd_queue[idx_cmd].v*np.sin(state_est[2])*dt
            state_est[2] = state_est[2] + self.cmd_queue[idx_cmd].omega * dt


            time_est = time_est + dt
            idx_cmd += 1

        # make sure covariance matrix is symmetric
        cov_est = 0.5*(cov_est + cov_est.T)

        return state_est, cov_est

    def FeedCommandQueue(self, v, omega, time):
        '''store applied commands in command queue'''
        self.command_queue.append(VehicleCommands(v,omega,time))


    def UpdateWithPoseMeasurement(self, x_meas, y_meas, theta_meas, cov_meas, time_meas):
        '''update state estimate with pose measurement'''

        # prior update
        state_prior, cov_prior = self.PredictState(time_meas, True)

        # remove old commands from queue
        while self.cmd_queue[0] < time_meas and len(self.cmd_queue) > 1:
            self.cmd_queue.popleft()

        # a posteriori update
        K = np.dot(cov_prior, np.linalg.inv(cov_prior + cov_meas))
        state_posterior = state_prior + np.dot(K, (np.array([x_meas, y_meas, theta_meas]) - state_prior))
        cov_posterior = cov_prior - np.dot(K, cov_prior)

        # store results
        self.state_est = state_posterior
        self.cov_est = cov_posterior