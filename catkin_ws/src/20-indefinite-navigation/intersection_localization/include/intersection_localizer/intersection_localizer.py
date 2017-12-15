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

        # edge detection parameters
        self.canny_lower_threshold = 100  # self.SetupParameter("~canny_lower_threshold", 100)
        self.canny_upper_threshold = 200  # self.SetupParameter("~canny_upper_threshold", 200)
        self.canny_aperture_size = 3  # self.SetupParameter("~canny_aperture_size", 5)

        # topview image parameters
        self.ppm = 800.0  # pixels per meters
        self.width = 0.6
        self.height = 0.4
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
<<<<<<< HEAD
        self.line_search_length = 30
        self.max_num_iter = 20
        self.ctrl_pts_density = 0.05  # number of control points per edge length (in pixels)
        self.min_num_ctrl_pts = 5
        self.max_num_ctrl_pts = 100
=======
        self.line_search_length = 20
        self.max_num_iter = 10
        self.ctrl_pts_density = 0.05  # number of control points per edge length (in pixels)
>>>>>>> b3aa25fdf370a7e9d48f154aa190740b54c6dc8e

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

        return img_processed

    def ApplyHomography(self, src):
        dst = np.zeros(shape=(2), dtype=float)
        dst[0] = (self.homography_01[0, 0] * src[0] + self.homography_01[0, 1] * src[1] + self.homography_01[0, 2]) / (
<<<<<<< HEAD
            self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])
        dst[1] = (self.homography_01[1, 0] * src[0] + self.homography_01[1, 1] * src[1] + self.homography_01[1, 2]) / (
            self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])

        return dst

    def ComputePose(self, img, x_init, y_init, theta_init, object_type):
        # initialize solution
=======
        self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])
        dst[1] = (self.homography_01[1, 0] * src[0] + self.homography_01[1, 1] * src[1] + self.homography_01[1, 2]) / (
        self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])

        return dst

    def ComputePose(self, img_processed, x_init, y_init, theta_init):
        for k in range(0, self.max_num_inter):
            pass

>>>>>>> b3aa25fdf370a7e9d48f154aa190740b54c6dc8e
        x_sol = x_init
        y_sol = y_init
        theta_sol = theta_init

<<<<<<< HEAD
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
            x_sol -= res[0]/self.ppm
            y_sol += res[1]/self.ppm
            theta_sol += res[2]

            # visualizer for debugging purposes
            if 1:
                img_color = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
                for i in range(0,len(idx_feasible_pts)):
                    pt_template = ctrl_pts_img_homogeneous[0:2,idx_feasible_pts[i]].astype(np.int)
                    pt_imag = (ctrl_pts_img_homogeneous[0:2,idx_feasible_pts[i]] + err[:,i]).astype(np.int)
                    cv2.circle(img_color, tuple(pt_template),2,(255,0,0), -1)
                    cv2.circle(img_color, tuple(pt_imag), 2, (0, 255, 0), -1)
                    cv2.line(img_color, tuple(pt_imag), tuple(pt_template), (0,0,255),1)

                cv2.imshow('img debug',img_color)
                cv2.waitKey(10)


        return True, x_sol, y_sol, theta_sol

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

        return not any(np.dot(A,pt_img_homogeneous) - b > 0.0)
=======
        return x_sol, y_sol, theta_sol
>>>>>>> b3aa25fdf370a7e9d48f154aa190740b54c6dc8e

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("%s = %s " % (param_name, value))
        return value
