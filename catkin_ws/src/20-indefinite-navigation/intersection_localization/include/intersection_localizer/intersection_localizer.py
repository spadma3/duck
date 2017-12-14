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
        self.line_search_length = 20
        self.max_num_iter = 10
        self.ctrl_pts_density = 0.05  # number of control points per edge length (in pixels)

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
        self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])
        dst[1] = (self.homography_01[1, 0] * src[0] + self.homography_01[1, 1] * src[1] + self.homography_01[1, 2]) / (
        self.homography_01[2, 0] * src[0] + self.homography_01[2, 1] * src[1] + self.homography_01[2, 2])

        return dst

    def ComputePose(self, img_processed, x_init, y_init, theta_init):
        for k in range(0, self.max_num_inter):
            pass

        x_sol = x_init
        y_sol = y_init
        theta_sol = theta_init

        return x_sol, y_sol, theta_sol

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("%s = %s " % (param_name, value))
        return value
