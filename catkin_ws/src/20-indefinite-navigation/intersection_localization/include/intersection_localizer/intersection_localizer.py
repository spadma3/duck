#!/usr/bin/env python
import rospy
import edge_model as em
import numpy as np
import duckietown_utils as dt
import cv2


class IntersectionLocalizer(object):
    '''class for localizing pose of Duckiebot at intersection'''

    def __init__(self, robot_name=''):
        self.robot_name = robot_name

        # camera parameters
        self.intrinsics = dt.load_camera_intrinsics(self.robot_name)
        self.homography_0 = dt.load_homography(self.robot_name)
        self.homography_0_inv = np.linalg.inv(self.homography_0)

        # edge detection parameters
        self.canny_lower_threshold = 100  # self.SetupParameter("~canny_lower_threshold", 100)
        self.canny_upper_threshold = 200  # self.SetupParameter("~canny_upper_threshold", 200)
        self.canny_aperture_size = 3  # self.SetupParameter("~canny_aperture_size", 5)

        # topview image parameters
        self.ppm = 800.0  # pixels per meters
        self.width = 0.6
        self.height = 0.6
        self.height_offset = 0.1
        self.width_px = int(self.width * self.ppm)
        self.height_px = int(self.height * self.ppm)
        self.homography_1 = np.array([[0.0, -self.ppm, int(self.width * self.ppm * 0.5)],
                                      [-self.ppm, 0.0, int((self.height + self.height_offset) * self.ppm)],
                                      [0.0, 0.0, 1.0]])
        self.homography_01 = np.dot(self.homography_1, self.homography_0)

        # edge mask
        self.edge_mask_init = False
        self.mask_width = 2

        # localization algorithm parameters
        self.line_search_length = 30
        self.max_num_iter = 10
        self.ctrl_pts_density = 0.05  # number of control points per edge length (in pixels)
        self.min_num_ctrl_pts = 10
        self.max_num_ctrl_pts = 100

        # edge templates
        self.three_way_intersection = em.EdgeModel('THREE_WAY_INTERSECTION', self.ppm, self.ctrl_pts_density)
        self.four_way_intersection = em.EdgeModel('FOUR_WAY_INTERSECTION', self.ppm, self.ctrl_pts_density)

    def ProcessRawImage(self, img_raw):
        # rectify image
        img_undistorted = dt.rectify(dt.rgb_from_ros(img_raw), self.intrinsics)

        # compute grayscale image
        img_gray = cv2.cvtColor(img_undistorted, cv2.COLOR_RGB2GRAY)

        # warp image to top view
        img_top_view = cv2.warpPerspective(img_gray, self.homography_01, (self.width_px, self.height_px))

        # detect edges
        img_canny = cv2.Canny(img_top_view, self.canny_lower_threshold, self.canny_upper_threshold,
                              apertureSize=self.canny_aperture_size)

        # pad image to avoid running into borders
        img_processed = cv2.copyMakeBorder(img_canny, self.line_search_length, self.line_search_length,
                                           self.line_search_length, self.line_search_length, cv2.BORDER_CONSTANT,
                                           value=0)

        # mask edges from original image boundaries
        if not self.edge_mask_init:
            self.edge_mask_init = True

            height, width = img_undistorted.shape[:2]
            pt_shift = np.array([self.line_search_length, self.line_search_length], dtype=float)
            pt1 = self.ApplyHomography(np.array([0, height], dtype=float)) + pt_shift
            pt1_delta = self.ApplyHomography(np.array([0, height - 10], dtype=float)) + pt_shift
            pt1_dir = (pt1_delta - pt1) / np.linalg.norm(pt1_delta - pt1)
            pt0 = pt1 + (self.line_search_length - pt1[0]) / pt1_dir[0] * pt1_dir

            pt2 = self.ApplyHomography(np.array([width, height], dtype=float)) + pt_shift
            pt2_delta = self.ApplyHomography(np.array([width, height - 10], dtype=float)) + pt_shift
            pt2_dir = (pt2_delta - pt2) / np.linalg.norm(pt2_delta - pt2)
            pt3 = pt2 + (self.line_search_length + self.width_px - pt2[0]) / pt2_dir[0] * pt2_dir

            self.edge_mask_pt0 = tuple(np.rint(pt0).astype(np.int))
            self.edge_mask_pt1 = tuple(np.rint(pt1).astype(np.int))
            self.edge_mask_pt2 = tuple(np.rint(pt2).astype(np.int))
            self.edge_mask_pt3 = tuple(np.rint(pt3).astype(np.int))

        img_processed = cv2.line(img_processed, self.edge_mask_pt0, self.edge_mask_pt1, 0, self.mask_width)
        img_processed = cv2.line(img_processed, self.edge_mask_pt1, self.edge_mask_pt2, 0, self.mask_width)
        img_processed = cv2.line(img_processed, self.edge_mask_pt2, self.edge_mask_pt3, 0, self.mask_width)

        return img_processed, img_gray

    def ApplyHomographyB(self, src):
        dst = np.zeros(shape=(2), dtype=float)

        dst[0] = (self.homography_0_inv[0, 0] * src[0] + self.homography_0_inv[0, 1] * src[1] + self.homography_0_inv[
            0, 2]) / (
                     self.homography_0_inv[2, 0] * src[0] + self.homography_0_inv[2, 1] * src[1] +
                     self.homography_0_inv[2, 2])
        dst[1] = (self.homography_0_inv[1, 0] * src[0] + self.homography_0_inv[1, 1] * src[1] + self.homography_0_inv[
            1, 2]) / (
                     self.homography_0_inv[2, 0] * src[0] + self.homography_0_inv[2, 1] * src[1] +
                     self.homography_0_inv[2, 2])

        return dst

    def ApplyHomography(self, src):
        dst = np.zeros(shape=(2), dtype=float)
        dst[0] = (self.homography_01[0, 0] * src[0] + self.homography_01[0, 1] * src[1] + self.homography_01[0, 2]) / (
            self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])
        dst[1] = (self.homography_01[1, 0] * src[0] + self.homography_01[1, 1] * src[1] + self.homography_01[1, 2]) / (
            self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])

        return dst

    def ComputePose(self, img, x_init, y_init, theta_init, object_type):
        # initialize solution
        x_sol = x_init
        y_sol = y_init
        theta_sol = theta_init

        # iterative least squares problem
        for k in range(0, self.max_num_iter):
            # compute locations of control points
            if object_type == 'THREE_WAY_INTERSECTION':
                ctrl_pts_homogeneous, ctrl_pts_n_perp = self.three_way_intersection.ComputeControlPointsLocation(x_sol,
                                                                                                                 y_sol,
                                                                                                                 theta_sol)

            elif object_type == 'FOUR_WAY_INTERSECTION':
                ctrl_pts_homogeneous, ctrl_pts_n_perp = self.three_way_intersection.ComputeControlPointsLocation(x_sol,
                                                                                                                 y_sol,
                                                                                                                 theta_sol)

            else:
                # TODO: throw error
                pass

            # convert pts from vehicle to image frame
            ctrl_pts_img_homogeneous = self.Veh2ImgHomogeneous(ctrl_pts_homogeneous)
            ctrl_pts_n_perp_img = self.Veh2ImgDir(ctrl_pts_n_perp)

            # compute error
            err, idx_feasible_pts = self.ComputeError(img, ctrl_pts_img_homogeneous, ctrl_pts_n_perp_img)

            if len(idx_feasible_pts) < self.min_num_ctrl_pts:
                return False, 0.0, 0.0, 0.0

            # compute relative displacement and rotation
            pts_center = np.mean(ctrl_pts_img_homogeneous[:, idx_feasible_pts], axis=1)
            pts_delta = ctrl_pts_img_homogeneous[:, idx_feasible_pts] - pts_center[:, np.newaxis]
            N = len(idx_feasible_pts)
            A = np.zeros(shape=(2 * N, 3))
            A[:, 0:2] = np.tile(np.eye(2), [N, 1])
            A[:, 2] = np.dot(np.matrix([[0, -1], [1, 0]]), pts_delta[0:2, :]).transpose().reshape(2 * N)
            b = err.transpose().reshape(2 * N)
            res = np.dot(np.linalg.pinv(A), b)

            # convert displacement back from image frame to inertial frame and update solution
            x_sol -= res[0] / self.ppm
            y_sol += res[1] / self.ppm
            theta_sol += res[2]

            # debugging
            if 0:
                img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                for i in range(0, len(idx_feasible_pts)):
                    pt_template = ctrl_pts_img_homogeneous[0:2, idx_feasible_pts[i]].astype(np.int)
                    pt_imag = (ctrl_pts_img_homogeneous[0:2, idx_feasible_pts[i]] + err[:, i]).astype(np.int)
                    cv2.circle(img_color, tuple(pt_template), 2, (255, 0, 0), -1)
                    cv2.circle(img_color, tuple(pt_imag), 2, (0, 255, 0), -1)
                    cv2.line(img_color, tuple(pt_imag), tuple(pt_template), (0, 0, 255), 1)

                cv2.imshow('img debug', img_color)
                cv2.waitKey(10)

        return True, x_sol, y_sol, theta_sol

    def ComputeErrorB(self, img, pts_img_homogeneous, n_perp_img):
        img_padded = cv2.copyMakeBorder(img, self.line_search_length, self.line_search_length,
                                        self.line_search_length, self.line_search_length, cv2.BORDER_CONSTANT,
                                        value=0)
        offset = np.array([self.line_search_length, self.line_search_length, 0.0])
        pts_img_homogeneous = pts_img_homogeneous + offset[:, np.newaxis]

        # randomly iterate over all pts until maximum number of feasible points is reached
        idx = np.arange(0, pts_img_homogeneous.shape[1], dtype=int)
        np.random.shuffle(idx)

        N = np.min([self.max_num_ctrl_pts, pts_img_homogeneous.shape[1]])
        err = np.zeros(shape=(2, N))
        num_feasible = 0
        idx_feasible_pts = []
        for i in range(0, N):
            for k in range(0, self.line_search_length):
                if img_padded[int(round(pts_img_homogeneous[1, i] + k * n_perp_img[1, i])),
                              int(round(pts_img_homogeneous[0, i] + k * n_perp_img[0, i]))]:
                    err[:, num_feasible] = k * n_perp_img[:, i]
                    idx_feasible_pts.append(i)
                    num_feasible += 1
                    break

                elif img_padded[int(round(pts_img_homogeneous[1, i] - k * n_perp_img[1, i])),
                                int(round(pts_img_homogeneous[0, i] - k * n_perp_img[0, i]))]:
                    err[:, num_feasible] = -k * n_perp_img[:, i]
                    idx_feasible_pts.append(i)
                    num_feasible += 1
                    break

        # delete empty entries
        err = err[:, 0:num_feasible]

        return err, idx_feasible_pts

    def ComputeError(self, img, pts_img_homogeneous, n_perp_img):
        # randomly iterate over all pts until maximum number of feasible points is reached
        idx = np.arange(0, pts_img_homogeneous.shape[1], dtype=int)
        np.random.shuffle(idx)

        err = np.zeros(shape=(2, self.max_num_ctrl_pts))
        num_visible = 0
        num_feasible = 0
        idx_feasible_pts = []
        for i in idx:
            if self.IsPointVisible(pts_img_homogeneous[:, i]):
                num_visible += 1

                for k in range(0, self.line_search_length):
                    if img[int(round(pts_img_homogeneous[1, i] + k * n_perp_img[1, i])),
                           int(round(pts_img_homogeneous[0, i] + k * n_perp_img[0, i]))]:
                        err[:, num_feasible] = k * n_perp_img[:, i]
                        idx_feasible_pts.append(i)
                        num_feasible += 1
                        break

                    elif img[int(round(pts_img_homogeneous[1, i] - k * n_perp_img[1, i])),
                             int(round(pts_img_homogeneous[0, i] - k * n_perp_img[0, i]))]:
                        err[:, num_feasible] = -k * n_perp_img[:, i]
                        idx_feasible_pts.append(i)
                        num_feasible += 1
                        break

            if num_visible == self.max_num_ctrl_pts:
                break

        # delete empty entries
        err = err[:, 0:num_feasible]

        return err, idx_feasible_pts

    def ComputeVisibleEdge(self, ptA, ptB):
        A = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]])
        b = np.array([0.0, 640.0, 0.0, 480.0])

        if not np.any(np.dot(A, ptA) - b > 0.0):
            if not np.any(np.dot(A, ptB) - b > 0.0):
                # both are visible
                return True, ptA, ptB

            else:
                # one is visible (there is always a solution)
                n = ptB - ptA
                An = np.dot(A, n)
                res = (b - np.dot(A, ptA)) / An

                l_max = np.min(res[An > 0.0])

                return True, ptA, ptA + l_max * n

        else:
            if not np.any(np.dot(A, ptB) - b > 0.0):
                # one is visible (there is always a solution)
                n = ptA - ptB
                An = np.dot(A, n)
                res = (b - np.dot(A, ptB)) / An

                l_max = np.min(res[An > 0.0])

                return True, ptB + l_max * n, ptB

            else:
                # none is visible (make sure there is a solution)
                n = ptA - ptB
                An = np.dot(A, n)
                res = (b - np.dot(A, ptB)) / An

                l_max = np.min([np.min(res[An > 0.0]), 1.0])
                l_min = np.max([np.max(res[An <= 0.0]), 0.0])

                if l_max < l_min:
                    return False, ptA, ptB
                else:
                    return True, ptB + l_max * n, ptB + l_min * n

    def GenerateControlPointsB(self, edges, ctrl_pts_density):
        # computer number of control points
        N = len(edges)
        ctrl_pts_per_edge = np.zeros(shape=(N), dtype=int)
        for i in range(0, N):
            ctrl_pts_per_edge[i] = np.floor(edges[i].length * ctrl_pts_density) + 1

        num_ctrl_pts = np.sum(ctrl_pts_per_edge, axis=0, dtype=int)

        # generate control points
        ctrl_pts = np.zeros(shape=(2, num_ctrl_pts), dtype=float)
        ctrl_pts_homogeneous = np.zeros(shape=(3, num_ctrl_pts), dtype=float)
        ctrl_pts_n_perp = np.zeros(shape=(2, num_ctrl_pts), dtype=float)
        k = 0
        for i in range(0, N):
            offset = (edges[i].length - (ctrl_pts_per_edge[i] - 1.0) / ctrl_pts_density) / 2.0

            for j in range(0, ctrl_pts_per_edge[i]):
                ctrl_pts[:, k] = edges[i].ptA + (offset + np.float(j) / ctrl_pts_density) * edges[
                    i].n_par
                ctrl_pts_n_perp[:, k] = edges[i].n_perp
                k += 1

        ctrl_pts_homogeneous[0:2, :] = ctrl_pts[:, :]
        ctrl_pts_homogeneous[2, :] = 1

        return ctrl_pts, ctrl_pts_homogeneous, ctrl_pts_n_perp, num_ctrl_pts

    def DrawB(self, img, x, y, theta, object_type):
        if object_type == 'THREE_WAY_INTERSECTION':
            edges = self.three_way_intersection.edges

        elif object_type == 'FOUR_WAY_INTERSECTION':
            edges = self.four_way_intersection.edges

        else:
            # TODO error
            pass

        img2 = img[:, :].copy()

        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        t = np.dot(R, np.array([x, y], dtype=float))
        edges_img = []
        for edge in edges:
            ptA = np.dot(R, edge.ptA / self.ppm) - t
            ptB = np.dot(R, edge.ptB / self.ppm) - t

            # convert to image coordinates
            retA = self.homography_0_inv[2, 0] * ptA[0] + self.homography_0_inv[2, 1] * ptA[1] + self.homography_0_inv[
                2, 2]
            retB = self.homography_0_inv[2, 0] * ptB[0] + self.homography_0_inv[2, 1] * ptB[1] + self.homography_0_inv[
                2, 2]

            if retA < 0.0:
                ptA_img = self.ApplyHomographyB(ptA)
            else:
                n_par = (ptA - ptB)
                l = -retB / (
                    self.homography_0_inv[2, 0] * n_par[0] + self.homography_0_inv[2, 1] * n_par[1] +
                    self.homography_0_inv[2, 2])
                ptA_img = self.ApplyHomographyB(ptB + 0.99 * l * n_par)

            if retB < 0.0:
                ptB_img = self.ApplyHomographyB(ptB)
            else:
                n_par = (ptB - ptA)
                l = -retA / (self.homography_0_inv[2, 0] * n_par[0] + self.homography_0_inv[2, 1] * n_par[1] +
                             self.homography_0_inv[2, 2])
                ptB_img = self.ApplyHomographyB(ptA + 0.99 * l * n_par)

            visible, ptA_img, ptB_img = self.ComputeVisibleEdge(ptA_img, ptB_img)
            if visible:
                ptC_img = 0.5 * (ptA_img + ptB_img)

                n_par_img = (ptB_img - ptA_img) / np.linalg.norm(ptB_img - ptA_img)
                n_perp_img = np.array([-n_par_img[1], n_par_img[0]], dtype=float)
                ptD_img = ptC_img + 20 * n_perp_img

                cv2.line(img, tuple(np.round(ptA_img).astype(np.int)), tuple(np.round(ptB_img).astype(np.int)), 255, 1)
                cv2.line(img, tuple(np.round(ptC_img).astype(np.int)), tuple(np.round(ptD_img).astype(np.int)), 0, 1)
                cv2.circle(img, tuple(np.round(ptA_img).astype(np.int)), 3, 255, -1)
                cv2.circle(img, tuple(np.round(ptB_img).astype(np.int)), 3, 255, -1)

                edges_img.append(em.Edge(ptA_img, ptB_img))

        ctrl_pts, ctrl_pts_homogeneous, ctrl_pts_n_perp, num_ctrl_pts = self.GenerateControlPointsB(edges_img,
                                                                                                    self.ctrl_pts_density)
        for i in range(0, ctrl_pts.shape[1]):
            cv2.circle(img, tuple(np.round(ctrl_pts[:, i]).astype(np.int)), 3, 0, -1)

        # compute error
        img_canny = cv2.Canny(img2, self.canny_lower_threshold, self.canny_upper_threshold,
                              apertureSize=self.canny_aperture_size)
        cv2.imshow('edge', img_canny)
        err, idx_feasible_pts = self.ComputeErrorB(img_canny, ctrl_pts_homogeneous, ctrl_pts_n_perp)
        for i in range(0, len(idx_feasible_pts)):
            ptA = ctrl_pts[:, idx_feasible_pts[i]]
            ptB = ctrl_pts[:, idx_feasible_pts[i]] + err[:, i]
            cv2.line(img, tuple(np.round(ptA).astype(np.int)), tuple(np.round(ptB).astype(np.int)), 200, 1)
            cv2.circle(img, tuple(np.round(ctrl_pts[:, idx_feasible_pts[i]]).astype(np.int)), 3, 100, -1)

        # compute relative displacement and rotation
        pts_center = np.mean(ctrl_pts_homogeneous[:, idx_feasible_pts], axis=1)
        pts_delta = ctrl_pts_homogeneous[:, idx_feasible_pts] - pts_center[:, np.newaxis]
        N = len(idx_feasible_pts)
        A = np.zeros(shape=(2 * N, 3))
        A[:, 0:2] = np.tile(np.eye(2), [N, 1])
        A[:, 2] = np.dot(np.matrix([[0, -1], [1, 0]]), pts_delta[0:2, :]).transpose().reshape(2 * N)
        b = err.transpose().reshape(2 * N)
        res = np.dot(np.linalg.pinv(A), b)

    def DrawB2(self, img, x, y, theta, object_type):
        if object_type == 'THREE_WAY_INTERSECTION':
            edges = self.three_way_intersection.edges
            A_scale = np.array([[1.0/self.ppm, 0.0, 0.0],[0.0, 1.0/self.ppm, 0.0],[0.0, 0.0, 1.0]],dtype=float)
            ctrl_pts_h = np.dot(A_scale, self.three_way_intersection.ctrl_pts_homogeneous)
            ctrl_pts_n_par = self.three_way_intersection.ctrl_pts_n_par
            ctrl_pts_n_perp = self.three_way_intersection.ctrl_pts_n_perp
        elif object_type == 'FOUR_WAY_INTERSECTION':
            edges = self.four_way_intersection.edges

        else:
            # TODO error
            pass

        # project control points
        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        t = np.dot(R, np.array([x, y], dtype=float))

        T = np.zeros(shape=(3, 3), dtype=np.float)
        T[0:2, 0:2] = R
        T[0:2, 2] = -t
        T[2, 2] = 1
        ctrl_pts_h_img = np.dot(np.dot(self.homography_0_inv, T), ctrl_pts_h)
        feasible = ctrl_pts_h_img[2, :] < 0.0
        feasible_idx = np.arange(0, ctrl_pts_h.shape[1])[feasible]
        ctrl_pts_img = ctrl_pts_h_img[0:2, feasible] / ctrl_pts_h_img[2, feasible]

        # compute visible points
        A = np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]])
        b = np.array([0.0, 640.0, -120.0, 480.0])
        res = np.dot(A, ctrl_pts_img) - b[:, np.newaxis]
        visible = np.all(res < 0.0, 0)

        ctrl_pts_h = ctrl_pts_h[:, feasible_idx[visible]]
        ctrl_pts_h_img = ctrl_pts_h_img[:, feasible_idx[visible]]
        ctrl_pts_img = ctrl_pts_img[:, visible]

        # compute edge normal
        ctrl_pts_n_par_h_img = np.dot(np.dot(self.homography_0_inv[:, 0:2], R),
                                       ctrl_pts_n_par[:, feasible_idx[visible]])
        ctrl_pts_n_par_img = ctrl_pts_n_par_h_img[0:2, :] / ctrl_pts_n_par_h_img[2, :]
        A_par_to_perp = np.array([[0.0, 1.0],[-1.0,0.0]], dtype=float)
        N = np.dot(A_par_to_perp,(ctrl_pts_n_par_img - ctrl_pts_img))
        N = N / np.linalg.norm(N, 2, 0)

        ctrl_pts_n_perp_img = np.dot(np.dot(self.homography_0_inv[:, 0:2], R),
                                       ctrl_pts_n_perp[:, feasible_idx[visible]])
        ctrl_pts_n_perp_img = ctrl_pts_n_perp_img[0:2, :] / ctrl_pts_n_perp_img[2, :]
        N2 = ctrl_pts_n_perp_img - ctrl_pts_img
        N2 = N2 / np.linalg.norm(N2, 2, 0)

        #N = N2


        # compute L1
        G1 = np.zeros(shape=(3, 3), dtype=float)
        G1[0, 2] = 1
        ctrl_pts_h_img_1 = np.dot(np.dot(self.homography_0_inv, np.dot(G1, T)), ctrl_pts_h)
        ctrl_pts_img_1 = ctrl_pts_h_img_1[0:2, :] / ctrl_pts_h_img_1[2, :]
        L1 = (ctrl_pts_img_1 - ctrl_pts_img) * (ctrl_pts_h_img_1[2, :] / ctrl_pts_h_img[2, :])
        f1 = np.einsum('ij,ij->j', L1, N)

        # compute L2
        G2 = np.zeros(shape=(3, 3), dtype=float)
        G2[1, 2] = 1
        ctrl_pts_h_img_2 = np.dot(np.dot(self.homography_0_inv, np.dot(G2, T)), ctrl_pts_h)
        ctrl_pts_img_2 = ctrl_pts_h_img_2[0:2, :] / ctrl_pts_h_img_2[2, :]
        L2 = (ctrl_pts_img_2 - ctrl_pts_img) * (ctrl_pts_h_img_2[2, :] / ctrl_pts_h_img[2, :])
        f2 = np.einsum('ij,ij->j', L2, N)

        # compute L3
        G3 = np.zeros(shape=(3, 3), dtype=float)
        G3[0, 1] = 1
        G3[1, 0] = -1
        ctrl_pts_h_img_3 = np.dot(np.dot(self.homography_0_inv, np.dot(G3, T)), ctrl_pts_h)
        ctrl_pts_img_3 = ctrl_pts_h_img_3[0:2, :] / ctrl_pts_h_img_3[2, :]
        L3 = (ctrl_pts_img_3 - ctrl_pts_img) * (ctrl_pts_h_img_3[2, :] / ctrl_pts_h_img[2, :])
        f3 = np.einsum('ij,ij->j', L3, N)

        # compute distance to edge along edge normal
        img_canny = cv2.Canny(img, self.canny_lower_threshold, self.canny_upper_threshold,
                              apertureSize=self.canny_aperture_size)
        img_padded = cv2.copyMakeBorder(img_canny, self.line_search_length, self.line_search_length,
                                        self.line_search_length, self.line_search_length, cv2.BORDER_CONSTANT,
                                        value=0)
        offset = np.array([self.line_search_length, self.line_search_length])
        ctrl_pts_img_offset = ctrl_pts_img + offset[:, np.newaxis]

        # randomly iterate over all pts until maximum number of feasible points is reached
        idx = np.arange(0, ctrl_pts_img_offset.shape[1], dtype=int)
        np.random.shuffle(idx)

        num_pts = np.min([self.max_num_ctrl_pts, ctrl_pts_img_offset.shape[1]])
        D = np.zeros(shape=(num_pts), dtype=float)
        num_feasible = 0
        idx_feasible_pts = []
        for i in range(0, num_pts):
            for k in range(0, self.line_search_length):
                if img_padded[int(round(ctrl_pts_img_offset[1, i] + k * N[1, i])),
                              int(round(ctrl_pts_img_offset[0, i] + k * N[0, i]))]:
                    D[num_feasible] = k
                    idx_feasible_pts.append(i)
                    num_feasible += 1
                    break

                elif img_padded[int(round(ctrl_pts_img_offset[1, i] - k * N[1, i])),
                                int(round(ctrl_pts_img_offset[0, i] - k * N[0, i]))]:
                    D[num_feasible] = -k
                    idx_feasible_pts.append(i)
                    num_feasible += 1
                    break

        D = D[0:num_feasible]

        # compute weights
        S = 1.0/(np.abs(D) + 1.0)

        # solve least squares problems
        v = np.zeros(shape=(3), dtype=float)
        v[0] = np.dot(S*f1[idx_feasible_pts], D)
        v[1] = np.dot(S*f2[idx_feasible_pts], D)
        v[2] = np.dot(S*f3[idx_feasible_pts], D)
        C = np.zeros(shape=(3, 3), dtype=float)
        C[0, 0] = np.dot(S*f1[idx_feasible_pts], f1[idx_feasible_pts])
        C[0, 1] = np.dot(S*f1[idx_feasible_pts], f2[idx_feasible_pts])
        C[0, 2] = np.dot(S*f1[idx_feasible_pts], f3[idx_feasible_pts])
        C[1, 0] = C[0, 1]
        C[1, 1] = np.dot(S*f2[idx_feasible_pts], f2[idx_feasible_pts])
        C[1, 2] = np.dot(S*f2[idx_feasible_pts], f3[idx_feasible_pts])
        C[2, 0] = C[0, 2]
        C[2, 1] = C[1, 2]
        C[2, 2] = np.dot(S*f3[idx_feasible_pts], f3[idx_feasible_pts])
        res = np.dot(np.linalg.inv(C), v)

        for i in range(0, ctrl_pts_img.shape[1]):
            cv2.circle(img, tuple(np.round(ctrl_pts_img[:, i]).astype(np.int)), 2, 255, -1)
            ptA = ctrl_pts_img[:, i]
            ptB = ctrl_pts_img[:, i] + 20 * N[:, i]
            cv2.line(img, tuple(np.round(ptA).astype(np.int)), tuple(np.round(ptB).astype(np.int)), 155, 1)

            cv2.circle(img_canny, tuple(np.round(ctrl_pts_img[:, i]).astype(np.int)), 2, 255, -1)

        for i in range(0,num_feasible):
            ptA = ctrl_pts_img[:, idx_feasible_pts[i]]
            ptB = ctrl_pts_img[:, idx_feasible_pts[i]] +  D[i] * N[:, idx_feasible_pts[i]]
            cv2.circle(img_canny, tuple(np.round(ptB).astype(np.int)), 2, 255, -1)
            cv2.line(img_canny, tuple(np.round(ptA).astype(np.int)), tuple(np.round(ptB).astype(np.int)), 155, 1)

            '''ptA = ctrl_pts_img[:, idx_feasible_pts[i]]
            #ptB = ctrl_pts_img[:, idx_feasible_pts[i]] + 0.01 * L2[:, idx_feasible_pts[i]]
            ptB = ctrl_pts_img[:, idx_feasible_pts[i]] + (10.0/180.0*np.pi)*L3[:, idx_feasible_pts[i]]
            cv2.line(img_canny, tuple(np.round(ptA).astype(np.int)), tuple(np.round(ptB).astype(np.int)), 155, 1)'''
            cv2.imshow('canny', img_canny)

        return res[0], res[1], res[2]

    def Draw(self, img, x, y, theta, object_type):
        if object_type == 'THREE_WAY_INTERSECTION':
            edges = self.three_way_intersection.edges

        elif object_type == 'FOUR_WAY_INTERSECTION':
            edges = self.four_way_intersection.edges

        else:
            # TODO error
            pass

        R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
        t = np.dot(R, np.array([x, y], dtype=float) * self.ppm)
        for edge in edges:
            ptA = np.dot(R, edge.ptA) - t
            ptB = np.dot(R, edge.ptB) - t

            # convert to image coordinates
            ptA_img = self.Veh2Img(ptA, True)
            ptB_img = self.Veh2Img(ptB, True)

            cv2.line(img, tuple(ptA_img), tuple(ptB_img), 100, 1)

    def Veh2Img(self, pts_veh, as_int=False):
        A = np.array([[0.0, -1.0], [-1.0, 0.0]])

        if as_int:
            return (np.dot(A, pts_veh) + np.array([int(self.width * self.ppm * 0.5 + self.line_search_length), int(
                (self.height + self.height_offset) * self.ppm + self.line_search_length)])).astype(np.int)
        else:
            return (np.dot(A, pts_veh) + np.array([int(self.width * self.ppm * 0.5 + self.line_search_length), int(
                (self.height + self.height_offset) * self.ppm + self.line_search_length)]))

    def Veh2ImgHomogeneous(self, pts_veh_homogeneous, as_int=False):
        A = np.array([[0.0, -1.0, int(self.width * self.ppm * 0.5 + self.line_search_length)],
                      [-1.0, 0.0, int((self.height + self.height_offset) * self.ppm
                                      + self.line_search_length)], [0.0, 0.0, 1.0]])
        if as_int:
            return np.dot(A, pts_veh_homogeneous).astype(np.int)
        else:
            return np.dot(A, pts_veh_homogeneous)

    def Veh2ImgDir(self, n_veh, as_int=False):
        A = np.array([[0.0, -1.0], [-1.0, 0.0]])

        if as_int:
            return np.dot(A, n_veh).astype(np.int)
        else:
            return np.dot(A, n_veh)

    def IsPointVisible(self, pt_img_homogeneous):
        A = np.array([[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 1.0, 0.0]])
        b = np.array([-self.line_search_length, self.line_search_length + self.width_px, -self.line_search_length,
                      self.line_search_length + self.height_px])

        return not any(np.dot(A, pt_img_homogeneous) - b > 0.0)

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("%s = %s " % (param_name, value))
        return value
