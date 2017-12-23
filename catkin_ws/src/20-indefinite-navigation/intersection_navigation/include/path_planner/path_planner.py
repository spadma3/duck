#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import duckietown_utils as dt
import cv2


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
        self.coeffs_num = np.polysub(np.polymul(self.coeffs_vel[0, :], self.coeffs_acc[1, :]),
                                     np.polymul(self.coeffs_vel[1, :], self.coeffs_acc[0, :]))
        self.coeffs_denom = np.polyadd(np.polymul(self.coeffs_vel[0, :], self.coeffs_vel[0, :]),
                                       np.polymul(self.coeffs_vel[1, :], self.coeffs_vel[1, :]))

        # roots for minimizing curvature
        self.roots_init = False

    def Evaluate(self, s):
        pos = np.array([np.polyval(self.coeffs_pos[0, :], s), np.polyval(self.coeffs_pos[1, :], s)])
        vel = np.array([np.polyval(self.coeffs_vel[0, :], s), np.polyval(self.coeffs_vel[1, :], s)])

        return pos, vel

    def EvaluateNumerator(self, s):
        return np.abs(np.polyval(self.coeffs_num, s))

    def EvaluateDenominator(self, s):
        return np.power(np.polyval(self.coeffs_denom, s), 1.5)

    def EvaluateCurvature(self, s):
        return np.abs(np.polyval(self.coeffs_num, s)) / np.power(np.polyval(self.coeffs_denom, s), 1.5)

    def ComputeRoots(self):
        self.roots_init = True

        coeffs_num_der = np.polyder(self.coeffs_num)
        self.roots_num = np.roots(coeffs_num_der)
        self.roots_num = self.roots_num[np.isreal(self.roots_num)]
        if self.roots_num.shape[0]:
            self.roots_num = self.roots_num[self.roots_num < 0.0]
        if self.roots_num.shape[0]:
            self.roots_num = self.roots_num[self.roots_num > 1.0]

        self.roots_num = np.sort(self.roots_num)

        coeffs_denom_der = np.polyder(self.coeffs_denom)
        self.roots_denom = np.roots(coeffs_denom_der)
        self.roots_denom = self.roots_denom[np.isreal(self.roots_denom)]
        if self.roots_denom.shape[0]:
            self.roots_denom = self.roots_denom[self.roots_denom < 0.0]
        if self.roots_denom.shape[0]:
            self.roots_denom = self.roots_denom[self.roots_denom > 1.0]

        self.roots_denom = np.sort(self.roots_denom)


class PathPlanner(object):
    '''class for planning paths for a Duckiebot to cross an intersection'''

    def __init__(self, robot_name=''):
        self.robot_name = robot_name

        # optimization parameters
        self.max_iterations = 20
        self.num_intervals = 32
        self.alpha_min = 0.1
        self.alpha_max = 2.0
        self.delta_alpha = 0.001
        self.init_grad_step_size = 0.005
        self.delta_increase = 1.1
        self.delta_decrease = 0.5
        self.maximum_feasible_curvature = 0.0  # TODO, also implement!

        # drawing parameters
        homography = dt.load_homography(self.robot_name)
        self.H = np.linalg.inv(homography)

    def PlanPath(self, pose_init, alphas_init, pose_final, alphas_final):
        pos_init = np.array([pose_init[0], pose_init[1]], dtype=float)
        dir_init = np.array([np.cos(pose_init[2]), np.sin(pose_init[2])], dtype=float)

        pos_final = np.array([pose_final[0], pose_final[1]], dtype=float)
        dir_final = np.array([np.cos(pose_final[2]), np.sin(pose_final[2])], dtype=float)

        best_curvature_max = 1e20
        for alpha_init in alphas_init:
            for alpha_final in alphas_final:

                grad_step_size = self.init_grad_step_size
                for k in range(0, self.max_iterations):
                    # generate path
                    path = Path(pos_init, dir_init, alpha_init, pos_final, dir_final, alpha_final)
                    path.ComputeRoots()

                    # evaluate numerator, denominator of curvature
                    s = np.linspace(0.0, 1.0, self.num_intervals + 1)
                    val_num = path.EvaluateNumerator(s)
                    val_num_roots = path.EvaluateNumerator(path.roots_num)
                    val_denom = path.EvaluateDenominator(s)
                    val_denom_roots = path.EvaluateDenominator(path.roots_denom)

                    # bound numerator, denominator
                    bound_num = np.zeros(shape=(1, self.num_intervals), dtype=float)
                    roots_num = np.append(path.roots_num, 10)
                    idx_roots_num = 0

                    bound_denom = np.zeros(shape=(1, self.num_intervals), dtype=float)
                    roots_denom = np.append(path.roots_denom, 10)
                    idx_roots_denom = 0
                    for i in range(0, self.num_intervals):
                        if roots_num[idx_roots_num] < s[i + 1]:
                            bound_num[0, i] = max([val_num[i], val_num[i + 1], val_num_roots[idx_roots_num]])
                            idx_roots_num += 1
                        else:
                            bound_num[0, i] = np.max(val_num[i:i + 2])

                        if roots_denom[idx_roots_denom] < s[i + 1]:
                            bound_denom[0, i] = min([val_denom[i], val_denom[i + 1], val_denom_roots[idx_roots_denom]])
                            idx_roots_denom += 1
                        else:
                            bound_denom[0, i] = np.min(val_denom[i:i + 2])

                    # compute (conservative) estimate of maximum curvature
                    idx_max = np.argmax(bound_num / bound_denom)

                    if k > 0:
                        curvature_max_old = curvature_max

                    s_max = 0.5 * (s[idx_max] + s[idx_max + 1])
                    curvature_max = path.EvaluateCurvature(s_max)

                    # compute derivative of curvature w.r.t. alpha
                    path_d_init = Path(pos_init, dir_init, alpha_init + self.delta_alpha, pos_final, dir_final,
                                       alpha_final)
                    curvature_d_init = path_d_init.EvaluateCurvature(s_max)

                    path_d_final = Path(pos_init, dir_init, alpha_init, pos_final, dir_final,
                                        alpha_final + self.delta_alpha)
                    curvature_d_final = path_d_final.EvaluateCurvature(s_max)

                    if k > 0:
                        if curvature_max < curvature_max_old:
                            grad_step_size = self.delta_increase * grad_step_size
                        else:
                            grad_step_size = self.delta_decrease * grad_step_size

                    jac_init = 1.0 / self.delta_alpha * (curvature_d_init - curvature_max)
                    alpha_init = min(max(alpha_init - grad_step_size * jac_init, self.alpha_min), self.alpha_max)
                    jac_final = 1.0 / self.delta_alpha * (curvature_d_final - curvature_max)
                    alpha_final = min(max(alpha_final - grad_step_size * jac_final, self.alpha_min), self.alpha_max)

                    if curvature_max < best_curvature_max:
                        best_curvature_max = curvature_max
                        self.path = path
                        self.curvature_max = curvature_max
                        self.alpha_init = alpha_init
                        self.alpha_final = alpha_final

    def EvaluatePath(self, s):
        return self.path.Evaluate(s)

    def DrawPath(self, img, pose_current):
        num_segments = 30
        s = np.linspace(0.0, 1.0, num_segments + 1)

        # compute points along path
        pts, _ = self.path.Evaluate(s)

        # compute motion matrix
        R = np.array(
            [[np.cos(pose_current[2]), np.sin(pose_current[2])], [-np.sin(pose_current[2]), np.cos(pose_current[2])]])
        t = np.dot(R, np.array([pose_current[0], pose_current[1]], dtype=float))

        # project points into image frame
        pts_h = np.zeros(shape=(3, num_segments + 1), dtype=float)
        pts_h[0:2, :] = np.dot(R, pts) - t[:, np.newaxis]
        pts_h[2, :] = 1

        # convert to image points
        pts_img_h = np.dot(self.H, pts_h)
        pts_img = pts_img_h[0:2, :] / pts_img_h[2, :]

        # draw segments
        for i in range(0, num_segments):
            if pts_img_h[2, i] < 0.0 and pts_img_h[2, i + 1] < 0.0:
                cv2.line(img, tuple(np.round(pts_img[:, i]).astype(np.int)),
                         tuple(np.round(pts_img[:, i + 1]).astype(np.int)), 255, 3)


if __name__ == '__main__':
    pos_init = np.array([0.4, -0.16])
    pos_final = np.array([0.508, 0.159])
    #pos_final = np.array([0.0508, 0.4])

    alphas_init = np.linspace(0.1, 1.5, 11)
    alphas_final = np.linspace(0.1, 1.5, 11)

    path_planner = PathPlanner()

    pose_init = [pos_init[0], pos_init[1], 0.9*np.pi / 2.0]
    pose_final = [pos_final[0], pos_final[1], 0.0*np.pi]
    path_planner.PlanPath(pose_init, alphas_init, pose_final, alphas_final)

    print(path_planner.EvaluatePath(0.0))
    print(path_planner.EvaluatePath(1.0))
    print(path_planner.curvature_max)
    print(path_planner.alpha_init)
    print(path_planner.alpha_final)

    s = np.linspace(0.0, 1.0, 20)

    pts, _ = path_planner.path.Evaluate(s)

    fig = plt.figure()
    plt.plot(pts[0, :], pts[1, :])
    plt.grid(True)
    #plt.axis('equal')
    plt.axis([-0.2, 0.6, -0.2, 0.6])

    plt.show()
