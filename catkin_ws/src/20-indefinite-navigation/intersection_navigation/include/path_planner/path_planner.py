#!/usr/bin/env python
import numpy as np


class Path(object):
    '''class describing path for Duckiebot'''

    def __init__(self, pos_init, dir_init, alpha_init, pos_final, dir_final, alpha_final):
        delta_pos = pos_final - pos_init - alpha_init * dir_init
        delta_vel = alpha_final * dir_final - alpha_init * dir_init

        # path coefficients
        self.coeffs_pos = np.zeros(shape=(2, 4), dtype=float)
        for i in range(0, 2):
            c_temp = np.dot(np.array([[-2.0, 1.0], [3.0, -1.0]]), np.array([delta_pos[i], delta_vel[i]]))
            self.coeffs_pos[i, 0] = c_temp[0]
            self.coeffs_pos[i, 1] = c_temp[1]

            self.coeffs_pos[i, 2] = alpha_init * dir_init[i]
            self.coeffs_pos[i, 3] = pos_init[i]

        self.coeffs_vel = np.dot(self.coeffs_pos[:, 0:3], np.diag([3.0, 2.0, 1.0]))
        self.coeffs_acc = np.dot(self.coeffs_vel[:, 0:2], np.diag([2.0, 1.0]))
        self.coeffs_jerk = self.coeffs_acc[:, 0]

        # curvature coefficients
        self.coeffs_num = np.polysub(np.polymul(self.coeffs_vel[0,:], self.coeffs_acc[1,:]),np.polymul(self.coeffs_vel[1,:], self.coeffs_acc[0,:]))
        self.coeffs_denom = np.polyadd(np.polymul(self.coeffs_vel[0,:], self.coeffs_vel[0,:]),np.polymul(self.coeffs_vel[1,:], self.coeffs_vel[1,:]))

        # roots for minimizing curvature
        coeffs_num_der = np.polyder(self.coeffs_num)
        self.roots_num = np.roots(coeffs_num_der)
        self.roots_num = self.roots_num[not np.iscomplex(self.roots_num)]
        if self.roots_num.shape[0]:
            self.roots_num = self.roots_num[self.roots_num < 0.0]
        if self.roots_num.shape[0]:
            self.roots_num = self.roots_num[self.roots_num > 1.0]

        self.roots_num = np.sort(self.roots_num)

        coeffs_denom_der = np.polyder(self.coeffs_denom)
        self.roots_denom = np.roots(coeffs_denom_der)
        self.roots_denom = self.roots_denom[not np.iscomplex(self.roots_denom)]
        if self.roots_denom.shape[0]:
            self.roots_denom = self.roots_denom[self.roots_denom < 0.0]
        if self.roots_denom.shape[0]:
            self.roots_denom = self.roots_denom[self.roots_denom > 1.0]

        self.roots_denom = np.sort(self.roots_denom)

    def Evaluate(self, s):
        pos = np.array([np.polyval(self.coeffs_pos[0, :], s), np.polyval(self.coeffs_pos[1, :], s)])
        vel = np.array([np.polyval(self.coeffs_vel[0, :], s), np.polyval(self.coeffs_vel[1, :], s)])

        return pos, vel

    def EvaluateCurvature(self, s):
        return np.abs(np.polyval(self.coeffs_num, s))/np.power(np.polyval(self.coeffs_denom, s),1.5)


class PathPlanner(object):
    '''class for planning paths for a Duckiebot to cross an intersection'''

    def __init__(self):
        pass

    def PlanPath(self, pose_init, pose_final):
        pass


if __name__ == '__main__':
    pos_init = np.array([2.0, 0.0])
    pos_final = np.array([-3.0, -1.0])

    dir_init = np.array([1.0, 0.0])
    dir_final = np.array([0.0, -1.0])

    alpha_init = 0.1
    alpha_final = 1.0

    path = Path(pos_init, dir_init, alpha_init, pos_final, dir_final, alpha_final)

    pos, vel = path.EvaluatePath(0.0)
    print(pos)
    print(vel)
