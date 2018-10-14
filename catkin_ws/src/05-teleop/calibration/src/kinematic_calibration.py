#!/usr/bin/env python
# -*- coding: utf-8 -*-

# first save .bag file as robot_name.bag in duckiefleet/calibrations/sysid


import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import os.path
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from duckietown_utils import (logger, get_duckiefleet_root)
from duckietown_utils import rgb_from_ros

import time
import os
import math
import matplotlib.pyplot as plt
import scipy.interpolate as interp
from scipy.optimize import curve_fit
from scipy.optimize import minimize
from scipy.integrate import odeint
from mpl_toolkits.mplot3d import Axes3D
import yaml
import pickle

PREPARE_CALIBRATION_DATA_FOR_OPTIMIZATION = False
EXPERIMENT_NAME_FOR_PICKLE = "0910_4.pckl"
CALIBRATION_FORMULATION_APPROACH = "combined"
"""
TODO

1) Current way: extract c from ramp, cl and tr from sine
                simulate the system with the resulting params: how does changing the weights affect?
2) add regularization
3) switch to single optimization

TO TRY
1) different solver options for minimize
"""
class experimentData():
    pass
class calib():
    def __init__(self):
        # Initialize the node with rospy
        rospy.init_node('command', anonymous=True)

        # defaults overwritten by paramminimize(self.objective, p0)
        self.robot_name = rospy.get_param("~veh")

        self.OBJECTIVE_FN_COUNTER = 0

        if PREPARE_CALIBRATION_DATA_FOR_OPTIMIZATION:
            # Load homography
            self.H = self.load_homography()
            # define PinholeCameraModel
            self.pcm_ = PinholeCameraModel()
            # load camera info
            self.cam = self.load_camera_info()
            # load information about duckietown chessboard
            self.board_ = self.load_board_info()

            # load joystick commands
            # self.joy_cmd_=self.get_joy_command()

            # load wheel commands
            self.wheels_cmd_=self.get_wheels_command()
            # load veh pose
            self.veh_pose_ = self.load_camera_pose_estimation()
            self.experiment_data_ = self.unpackData()

        self.y_pred_ramp_default_params = None
        self.y_opt_predict_ramp = None
        self.y_pred_sine_default_params = None
        self.y_opt_predict_sine = None
        self.y_opt_predict_combine = None

        self.fit_=self.nonlinear_model_fit()
        #self.plots()
        # write to the kinematic calibration file
        #self.write_calibration()

        # make plots & visualizations
        #self.plot=self.visualize()
        #plt.show()

    # wait until we have recieved the camera info message through ROS and then initialize
    def initialize_pinhole_camera_model(self, camera_info):
        self.ci_ = camera_info
        self.pcm_.fromCameraInfo(camera_info)
        print("pinhole camera model initialized")

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + self.robot_name + ".yaml")
        if not os.path.isfile(filename):
            logger.warn("no extrinsic calibration parameters for {}, trying default".format(self.robot_name))
            filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/default.yaml")
            if not os.path.isfile(filename):
                logger.error("can't find default either, something's wrong")
            else:
                data = yaml_load_file(filename)
        else:
            rospy.loginfo("Using extrinsic calibration of " + self.robot_name)
            data = yaml_load_file(filename)
        logger.info("Loaded homography for {}".format(os.path.basename(filename)))
        return np.array(data['homography']).reshape((3, 3))

    def load_camera_info(self):
        '''Load camera intrinsics'''
        filename = (os.environ['DUCKIEFLEET_ROOT'] + "/calibrations/camera_intrinsic/" + self.robot_name + ".yaml")
        if not os.path.isfile(filename):
            logger.warn("no intrinsic calibration parameters for {}, trying default".format(self.robot_name))
            filename = (os.environ['DUCKIEFLEET_ROOT'] + "/calibrations/camera_intrinsic/default.yaml")
            if not os.path.isfile(filename):
                logger.error("can't find default either, something's wrong")
        calib_data = yaml_load_file(filename)

        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = np.array(calib_data['camera_matrix']['data']).reshape((3,3))
        cam_info.D = np.array(calib_data['distortion_coefficients']['data']).reshape((1,5))
        cam_info.R = np.array(calib_data['rectification_matrix']['data']).reshape((3,3))
        cam_info.P = np.array(calib_data['projection_matrix']['data']).reshape((3,4))
        cam_info.distortion_model = calib_data['distortion_model']
        logger.info("Loaded camera calibration parameters for {} from {}".format(self.robot_name, os.path.basename(filename)))
        return cam_info

    def load_board_info(self, filename=''):
        '''Load calibration checkerboard info'''
        if not os.path.isfile(filename):
            filename = get_ros_package_path('duckietown') + '/config/baseline/ground_projection/ground_projection/default.yaml'
        target_data = yaml_load_file(filename)
        target_info = {
            'width': target_data['board_w'],
            'height': target_data['board_h'],
            'square_size': target_data['square_size'],
            'x_offset': target_data['x_offset'],
            'y_offset': target_data['y_offset'],
            'offset': np.array([target_data['x_offset'], -target_data['y_offset']]),
            'size': (target_data['board_w'], target_data['board_h']),
          }
        logger.info("Loaded checkerboard parameters")
        return target_info

    # This function draws lines joining the given image points to the first chess board corner
    def draw(self,img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return img

    def load_camera_pose_estimation(self):
        # initialize lists
        x,y,z,pitch,roll,yaw,timestamp =([] for i in range(7))

        pose_straight = {
            'x_veh': [],
            'y_veh': [],
            'z_veh': [],
            'yaw_veh':[],
            'roll_veh':[],
            'pitch_veh':[],
            'timestamp':[],
          }
        pose_curve = {
            'x_veh': [],
            'y_veh': [],
            'z_veh': [],
            'yaw_veh':[],
            'roll_veh':[],
            'pitch_veh':[],
            'timestamp':[],
          }

        # Define the chess board rows and columns
        chw=self.board_['width']
        chh=self.board_['height']
        patternSize = (chw, chh)
        chSeize=self.board_['square_size']

        # Set the termination criteria for the corner sub-pixel algorithm
        # if resolution is low keep iteration to minimum otherwise it messes up the code
        # the process of corner refinement stops either after critera.max.count iteration or when the corner moves by less than criteria.epsilon on some iteration
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,20,0.01)  # before was 30 instead of 20
        find_chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chw * chh, 3), np.float32)
        objp[:, :2] = np.mgrid[0:chw, 0:chh].T.reshape(-1, 2)
        objp = objp.reshape(-1, 1, 3)
        objp = np.multiply(objp, chSeize)

        # Create the axis points
        axisPoints = np.float32([[3*chSeize, 0, 0], [0, 3*chSeize, 0], [0, 0, -3*chSeize]]).reshape(-1, 3)

        #define the inputbag and topicname
        inputbag=rospy.get_param("~path")+self.robot_name+"_calibration.bag"
        topicname = "/" + self.robot_name + "/camera_node/image/compressed"
        indexcounter=1

        #initialize recording state
        Recording=False

        #get Index where there is no velocity command its respective timestamp
        stopIndex = [i for i, j in enumerate(self.wheels_cmd_['vel_r']) if j == 0]
        stopTime=[self.wheels_cmd_['timestamp'][i] for i in stopIndex]
        #get Index where there is a velocity command its respective timestamp
        starting_ind = [i for i, j in enumerate(self.wheels_cmd_['vel_r']) if j != 0]
        startTime=[self.wheels_cmd_['timestamp'][i] for i in starting_ind]
        counter=0
        count_checkerboard_scenes = 0


        # Loop over the image files contained in rosbag
        for topic, msg, t in rosbag.Bag(inputbag).read_messages(topics=topicname):
            indexcounter+=1
            dt = rospy.Duration(secs=1.0/30.) # start recording approx one frame before first wheel cmd

            for i in range(len(stopTime)):
                if abs(t - stopTime[i]) < dt:
                    Recording=False
            for i in range(len(startTime)):
                if abs(t - startTime[i]) < dt:
                    Recording = True

            if Recording:
                img = cv2.cvtColor(rgb_from_ros(msg), cv2.COLOR_BGR2RGB)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(img, patternSize, None,find_chessboard_flags)

                if ret == True:
                    counter=0 # reset counter
                    count_checkerboard_scenes += 1
                    print "checkerboard in frame %d found" %indexcounter
                    #rospy.loginfo("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx")
                    # Refine the corner position
                    corners2 = cv2.cornerSubPix(gray, corners, (5,5), (-1, -1), criteria)

                    # only reverse order if first point is at bottom right corner
                    if corners2[0][0][0]>corners2[34][0][0]:
                        #print "Reversing order of points."
                        corners2=corners2[::-1]

                    # Draw and display the corners
                    img = cv2.drawChessboardCorners(img, (chw, chh), corners2, ret)

                    # Find the rotation and translation vectors.
                    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, self.cam.K, np.array([]))

                    #Red Z-axis = pointing away from image
                    #Blue X-axis = pointing right or left
                    #Green Y-axis = pointing down or up

                    t_chess_cam=tvecs

                    # camera pose from object points
                    R_cam_chess = cv2.Rodrigues(rvecs)[0]
                    R_chess_cam=np.matrix(R_cam_chess).T
                    t_cam_chess =-np.matrix(R_cam_chess).T * np.matrix(tvecs)

                    T_chess_cam = np.zeros((4, 4))
                    T_chess_cam[3, 3] = 1
                    T_chess_cam[:3, :3] = R_chess_cam
                    T_chess_cam[:3, 3] = t_cam_chess.transpose()


                    # change direction of coordinates system
                    R_world_chess=np.matrix('0 0 1;-1 0 0;0 -1 0')

                    # R_world_chess=np.eye(3)
                    T_world_chess = np.zeros((4, 4))
                    T_world_chess[3, 3] = 1
                    T_world_chess[:3, :3] = R_world_chess
                    T_world_chess[:3, 3] = [0, 0, 0]

                    #translate coordinate cam_pose to vehicle pose
                    angle =0/180.0*math.pi #guess of camera angle in degree to veh; set to zero since results were not satisfying
                    R_cam_veh=self.ZYXEuler2rot([0,float(angle),0.0])

                    assert(self.isRotationMatrix(R_cam_veh))

                    T_cam_veh = np.zeros((4, 4))
                    T_cam_veh[3, 3] = 1
                    T_cam_veh[:3, :3] = R_cam_veh

                    T_world_veh=T_world_chess.dot(T_chess_cam).dot(T_cam_veh)

                    veh_pos = T_world_veh[:3, 3]
                    # get euler angles of vehicles with respect to world coordinate frame [x,y,z] in radians
                    veh_eulerangles=self.rot2ZYXEuler(T_world_veh[:3, :3])


                    # print "-------Euler Angles-------"
                    # print eulerangles

                    #split into 2 arrays for the different experiments
                    if t<stopTime[0]:
                        pose_straight['x_veh'].append(veh_pos[0])
                        pose_straight['y_veh'].append(veh_pos[1])
                        pose_straight['z_veh'].append(veh_pos[2])
                        pose_straight['roll_veh'].append(veh_eulerangles[0])
                        pose_straight['pitch_veh'].append(veh_eulerangles[1])
                        pose_straight['yaw_veh'].append(veh_eulerangles[2])
                        pose_straight['timestamp'].append(t)
                    else:
                        pose_curve['x_veh'].append(veh_pos[0])
                        pose_curve['y_veh'].append(veh_pos[1])
                        pose_curve['z_veh'].append(veh_pos[2])
                        pose_curve['roll_veh'].append(veh_eulerangles[0])
                        pose_curve['pitch_veh'].append(veh_eulerangles[1])
                        pose_curve['yaw_veh'].append(veh_eulerangles[2])
                        pose_curve['timestamp'].append(t)

                    # project 3D points to image plane
                    imgpts, jac = cv2.projectPoints(axisPoints, rvecs, tvecs, self.cam.K, self.cam.D)
                    img = self.draw(img, corners2, imgpts)
                    cv2.imshow('img', img)
                    cv2.waitKey(1)
                else:
                    print "checkerboard in frame %d not found" %indexcounter
                    #counter+=1
                    #if counter==15:
                    #    print "Finished recording"
                    #    Recording=False
                    #    break

        cv2.destroyAllWindows() #close window

        data_pre_filter ={'straight':pose_straight,'curve':pose_curve}
        rospy.loginfo("NUMBER OF SCENES WITH CHECKERBOARD Before Pose Filter: {}".format(str(count_checkerboard_scenes)))
        rospy.loginfo("""\nStraight Pose Data Before Pose Filter
                         Size : {data_pre_filter_struct_size}
                         Type : {data_pre_filter_struct_type}
                         Variables : {variables}
                         [Time Stamp (sec)] Size: {time_stamp_size} Duration: {duration}
                         [Global X] Size: {global_x_size} Max: {max_global_x} Min: {min_global_x}
                         [Global Y] Size: {global_y_size} Max: {max_global_y} Min: {min_global_y}
                         [Global Yaw] Size: {yaw_size} Max: {max_yaw} Min: {min_yaw}
                         """
                         .format(data_pre_filter_struct_size = len(data_pre_filter['straight']),
                                 data_pre_filter_struct_type = type(data_pre_filter['straight']),
                                 variables = data_pre_filter['straight'].keys(),
                                 time_stamp_size = len(data_pre_filter['straight']['timestamp']), duration = (max(data_pre_filter['straight']['timestamp']) - min(data_pre_filter['straight']['timestamp'])).to_sec(),
                                 global_x_size = len(data_pre_filter['straight']['x_veh']), max_global_x = max(data_pre_filter['straight']['x_veh']), min_global_x = min(data_pre_filter['straight']['x_veh']),
                                 global_y_size = len(data_pre_filter['straight']['y_veh']), max_global_y = max(data_pre_filter['straight']['y_veh']), min_global_y = min(data_pre_filter['straight']['y_veh']),
                                 yaw_size = len(data_pre_filter['straight']['yaw_veh']), max_yaw = max(data_pre_filter['straight']['yaw_veh']), min_yaw = min(data_pre_filter['straight']['yaw_veh'])
                                )
                     )
        rospy.loginfo("""\Curve Pose Data Before Pose Filter
                         Size : {data_pre_filter_struct_size}
                         Type : {data_pre_filter_struct_type}
                         Variables : {variables}
                         [Time Stamp] Size: {time_stamp_size} Extracted Frame's Duration: {duration}
                         [Global X] Size: {global_x_size} Max: {max_global_x} Min: {min_global_x}
                         [Global Y] Size: {global_y_size} Max: {max_global_y} Min: {min_global_y}
                         [Global Yaw] Size: {yaw_size} Max: {max_yaw} Min: {min_yaw}
                         """
                         .format(data_pre_filter_struct_size = len(data_pre_filter['curve']),
                                 data_pre_filter_struct_type = type(data_pre_filter['curve']),
                                 variables = data_pre_filter['curve'].keys(),
                                 time_stamp_size = len(data_pre_filter['curve']['timestamp']), duration = (max(data_pre_filter['curve']['timestamp']) - min(data_pre_filter['curve']['timestamp'])).to_sec(),
                                 global_x_size = len(data_pre_filter['curve']['x_veh']), max_global_x = max(data_pre_filter['curve']['x_veh']), min_global_x = min(data_pre_filter['curve']['x_veh']),
                                 global_y_size = len(data_pre_filter['curve']['y_veh']), max_global_y = max(data_pre_filter['curve']['y_veh']), min_global_y = min(data_pre_filter['curve']['y_veh']),
                                 yaw_size = len(data_pre_filter['curve']['yaw_veh']), max_yaw = max(data_pre_filter['curve']['yaw_veh']), min_yaw = min(data_pre_filter['curve']['yaw_veh'])
                                )
                     )

        #MA filter with window size N;
        filter=False
        if filter:
            N = 3
            pose_straight=self.poseFilter(pose_straight,N)
            pose_curve=self.poseFilter(pose_curve,N)

        data ={'straight':pose_straight,'curve':pose_curve}

        rospy.loginfo("NUMBER OF SCENES WITH CHECKERBOARD After Pose Filter: {}".format(str(count_checkerboard_scenes)))
        rospy.loginfo("""\nStraight Pose Data After Pose Filter
                         Size : {data_struct_size}
                         Type : {data_struct_type}
                         Variables : {variables}
                         [Time Stamp] Size: {time_stamp_size} Duration: {duration}
                         [Global X] Size: {global_x_size} Max: {max_global_x} Min: {min_global_x}
                         [Global Y] Size: {global_y_size} Max: {max_global_y} Min: {min_global_y}
                         [Global Yaw] Size: {yaw_size} Max: {max_yaw} Min: {min_yaw}
                         """
                         .format(data_struct_size = len(data['straight']),
                                 data_struct_type = type(data['straight']),
                                 variables = data['straight'].keys(),
                                 time_stamp_size = len(data['straight']['timestamp']), duration = (max(data_pre_filter['straight']['timestamp']) - min(data_pre_filter['straight']['timestamp'])).to_sec(),
                                 global_x_size = len(data['straight']['x_veh']), max_global_x = max(data['straight']['x_veh']), min_global_x = min(data['straight']['x_veh']),
                                 global_y_size = len(data['straight']['y_veh']), max_global_y = max(data['straight']['y_veh']), min_global_y = min(data['straight']['y_veh']),
                                 yaw_size = len(data['straight']['yaw_veh']), max_yaw = max(data['straight']['yaw_veh']), min_yaw = min(data['straight']['yaw_veh'])
                                )
                     )
        rospy.loginfo("""\nCurve Pose Data After Pose Filter
                         Size : {data_struct_size}
                         Type : {data_struct_type}
                         Variables : {variables}
                         [Time Stamp] Size: {time_stamp_size} Duration: {duration}
                         [Global X] Size: {global_x_size} Max: {max_global_x} Min: {min_global_x}
                         [Global Y] Size: {global_y_size} Max: {max_global_y} Min: {min_global_y}
                         [Global Yaw] Size: {yaw_size} Max: {max_yaw} Min: {min_yaw}
                         """
                         .format(data_struct_size = len(data['curve']),
                                 data_struct_type = type(data['curve']),
                                 variables = data['curve'].keys(),
                                 time_stamp_size = len(data['curve']['timestamp']), duration = (max(data_pre_filter['curve']['timestamp']) - min(data_pre_filter['curve']['timestamp'])).to_sec(),
                                 global_x_size = len(data['curve']['x_veh']), max_global_x = max(data['curve']['x_veh']), min_global_x = min(data['curve']['x_veh']),
                                 global_y_size = len(data['curve']['y_veh']), max_global_y = max(data['curve']['y_veh']), min_global_y = min(data['curve']['y_veh']),
                                 yaw_size = len(data['curve']['yaw_veh']), max_yaw = max(data['curve']['yaw_veh']), min_yaw = min(data['curve']['yaw_veh'])
                                )
                     )

        #rospy.loginfo("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxx")
        #rospy.loginfo(data['straight']['timestamp'])
        return data

    # Get joystick command
    def get_joy_command(self):
        omg=[]
        vel=[]
        timestamp=[]
        inputbag=rospy.get_param("~path")+self.robot_name+"_calibration.bag"

        topicname = "/" + self.robot_name + "/joy_mapper_node/car_cmd"
        for topic, msg, t in rosbag.Bag(inputbag).read_messages(topics=topicname):
            vel.append(msg.v)
            omg.append(msg.omega)
            timestamp.append(t)

        cmd = {
            'velocity': vel,
            'omega': omg,
            'timestamp': timestamp,
        }
        return cmd

    # Get wheels command
    def get_wheels_command(self):
        vel_l = []
        vel_r = []
        timestamp = []
        inputbag = rospy.get_param("~path")+self.robot_name+"_calibration.bag"
        topicname = "/" + self.robot_name + "/wheels_driver_node/wheels_cmd"

        for topic, msg, t in rosbag.Bag(inputbag).read_messages(topics=topicname):
            vel_l.append(msg.vel_left)
            vel_r.append(msg.vel_right)
            timestamp.append(t)

        cmd = {
            'vel_r': vel_r,
            'vel_l': vel_l,
            'timestamp': timestamp,
        }
        return cmd

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-3

    def listFilter(self,x,N):
        x = np.convolve(x, np.ones((N,)) / N, mode='valid')
        return x

    def poseFilter(self,pose,N):
        for key in pose:
            print key
            if key != "timestamp":
                pose[key]= np.reshape(pose[key], len(pose[key]))
                pose[key]=np.convolve(pose[key], np.ones((N,)) / N, mode='valid')
        return pose

    # ZYX Euler Angles
    # Yaw: around z axis -> pitch around y-axis-> roll around x-axis
    # should exactly match the output of MATLABs rotm2euler but the order of x and z are swapped
    def rot2ZYXEuler(self,R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    # Calculates Rotation Matrix given euler angles.
    def ZYXEuler2rot(self,theta):

        R_x = np.array([[1.0, 0.0, 0.0],
                        [0.0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0.0, math.sin(theta[0]), math.cos(theta[0])]
                        ])

        R_y = np.array([[math.cos(theta[1]), 0.0, math.sin(theta[1])],
                        [0.0, 1.0, 0.0],
                        [-math.sin(theta[1]), 0.0, math.cos(theta[1])]
                        ])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0.0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0.0],
                        [0.0, 0.0, 1.0]
                        ])

        R = np.dot(R_z, np.dot(R_y, R_x))

        return R

    def compressArray(self,ref_size,arr):
        arr_interp = interp.interp1d(np.arange(arr.size), arr)
        arr_compress = arr_interp(np.linspace(0, arr.size - 1, ref_size))
        return arr_compress

    def unpackData(self):
        ### RAMP UP COMMAND EXPERIMENT
        ## Measuremet States
        # unpack the measured states -> model output
        x_ramp_meas    = self.veh_pose_['straight']['x_veh']
        x_ramp_meas    = np.reshape(x_ramp_meas,np.size(x_ramp_meas))  # fixing some totally fucked up dimensions
        y_ramp_meas    = self.veh_pose_['straight']['y_veh']
        y_ramp_meas    = np.reshape(y_ramp_meas,np.size(y_ramp_meas))  # fixing some totally fucked up dimensions
        yaw_ramp_meas  = self.veh_pose_['straight']['yaw_veh']
        yaw_ramp_meas  = ( np.array(yaw_ramp_meas) + np.pi*0.5 + np.pi) % (2 * np.pi ) - np.pi # shift by pi/2 and wrap to +-pi

        # unpack the the array containing time instances of the mentioned measurements
        time_ramp_meas = self.veh_pose_['straight']['timestamp']
        for i in range(0,np.size(time_ramp_meas)):  # convert to float
            time_ramp_meas[i] = time_ramp_meas[i].to_sec()
        time_ramp_meas = np.array(time_ramp_meas)

        ## Input Commands
        cmd_ramp_right = np.concatenate([[0],self.wheels_cmd_['vel_r']])  # add a 0 cmd at the beginning
        cmd_ramp_left  = np.concatenate([[0],self.wheels_cmd_['vel_l']])  # as interpolation boundary

        # unpack the the array containing time instances of the mentioned measurements
        time_ramp_cmd  = self.wheels_cmd_['timestamp']
        for i in range(0,np.size(time_ramp_cmd)):  # convert time to float
            time_ramp_cmd[i] = time_ramp_cmd[i].to_sec()
        time_ramp_cmd = np.array(time_ramp_cmd)

        ### SINE COMMAND EXPERIMENT
        ## Measuremet States
        # unpack the measured states -> model output
        x_sine_meas    = self.veh_pose_['curve']['x_veh']
        x_sine_meas    = np.reshape(x_sine_meas,np.size(x_sine_meas))  # fixing some totally fucked up dimensions
        y_sine_meas    = self.veh_pose_['curve']['y_veh']
        y_sine_meas    = np.reshape(y_sine_meas,np.size(y_sine_meas))  # fixing some totally fucked up dimensions
        yaw_sine_meas  = self.veh_pose_['curve']['yaw_veh']
        yaw_sine_meas  = ( np.array(yaw_sine_meas) + np.pi*0.5 + np.pi) % (2 * np.pi ) - np.pi # shift by pi/2 and wrap to +-pi

        # unpack the the array containing time instances of the mentioned measurements
        time_sine_meas = self.veh_pose_['curve']['timestamp']
        for i in range(0,np.size(time_sine_meas)):  # convert time to float
            time_sine_meas[i] = time_sine_meas[i].to_sec()
        time_sine_meas = np.array(time_sine_meas)

        cmd_sine_right = np.concatenate([[0],self.wheels_cmd_['vel_r']])  # add a 0 cmd at the beginning
        cmd_sine_left  = np.concatenate([[0],self.wheels_cmd_['vel_l']])  # as interpolation boundary
        time_sine_cmd  = self.wheels_cmd_['timestamp']

        timepoints_ramp, time_ramp, cmd_ramp_right, cmd_ramp_left, timepoints_sine, time_sine, cmd_sine_right, cmd_sine_left = self.resampling(time_ramp_meas, time_ramp_cmd, cmd_ramp_right, cmd_ramp_left, time_sine_meas, time_sine_cmd, cmd_sine_right, cmd_sine_left)

        experimentDataObj = experimentData()
        experimentDataObj.x_ramp_meas = x_ramp_meas
        experimentDataObj.y_ramp_meas = y_ramp_meas
        experimentDataObj.yaw_ramp_meas = yaw_ramp_meas

        experimentDataObj.x_sine_meas = x_sine_meas
        experimentDataObj.y_sine_meas = y_sine_meas
        experimentDataObj.yaw_sine_meas = yaw_sine_meas

        experimentDataObj.timepoints_ramp = timepoints_ramp
        experimentDataObj.time_ramp = time_ramp

        experimentDataObj.cmd_ramp_right = cmd_ramp_right
        experimentDataObj.cmd_ramp_left = cmd_ramp_left

        experimentDataObj.timepoints_sine = timepoints_sine
        experimentDataObj.time_sine = time_sine

        experimentDataObj.cmd_sine_right = cmd_sine_right
        experimentDataObj.cmd_sine_left = cmd_sine_left

        #print 'XXXXXXXXXXXXXXXXXXXXXXXXX'
        #print experimentDataObj

        f = open(EXPERIMENT_NAME_FOR_PICKLE, 'wb')
        pickle.dump(experimentDataObj, f)
        f.close()

        return experimentDataObj

    def simulation_model(self, X, c, cl, tr, x_0, y_0, yaw_0, d):
        Ts=1.0/30 # Sampling Time of Euler Integration step

        d = 0.06  # Distance of camera from Baseline is fixed and not part of the optimization for now
        #X=X.reshape((int(X.size/2),2),order='F')
        #cmd_right = X[:,0]
        #cmd_left  = X[:,1]
        cmd_size  = np.int(X[-1])  # This is not so elegant but I had no better idea for now
        cmd_right = X[0:cmd_size]
        cmd_left  = X[cmd_size:cmd_size*2]
        timepoints= X[cmd_size*2:-1].astype(int)

        # Model based on parameters predicts velocities and turn rates
        # c is the "forward speed gain"
        # cl is the "turning rate gain"
        # tr is the "undesired turning rate gain when commanding to go straight"
        # This model deliberately neglects the influence of the tr on the forward speed
        # d is the offset of the camera to center of the wheels
        vx_pred = c * (cmd_right+cmd_left)* 0.5 # + tr * (cmd_right-cmd_left)*0.5
        omega_pred = cl * (cmd_right-cmd_left) * 0.5 + tr * (cmd_right+cmd_left) * 0.5
        vy_pred = omega_pred*d  # The model currently also estimates the offset of the camera position

        # Forward Euler Integration (improve to RK4?) to get Position Estimates
        yaw_pred = np.cumsum(omega_pred)*Ts + yaw_0
        x_pred = np.cumsum(np.cos(yaw_pred)*vx_pred + np.sin(yaw_pred)*vy_pred)*Ts + x_0
        y_pred = np.cumsum(np.sin(yaw_pred)*vx_pred + np.cos(yaw_pred)*vy_pred)*Ts + y_0

        # The _0 Parameters are just the integration constants and also need to be fitted
        # but we don't care about them later

        # Match and pick the prediction steps with the closest available position
        # Based on looking at the data we assume the images were taken at a perfect
        # 30 FPS rate. Therefore, we match to the fixed (perfect) sampling rate
        # and do not interpolate. The pred vectors are thus evaluated at the timepoints

        # Output has to be a 1D Vector for curve_fit to work
        Y = np.concatenate((x_pred[timepoints], y_pred[timepoints], yaw_pred[timepoints]), axis=0)
        return Y

    @staticmethod
    def kinematic_model(s, t,cmd_right,cmd_left, p):
        d = 0.06  # Distance of camera from Baseline is fixed and not part of the optimization for now
        # optimized parameters
        c, cl, tr = p

        # Simple Kinematic model
        # Assumptions: Rigid body, No lateral slip

        x_dot_rob = c * (cmd_right+cmd_left)* 0.5 # + tr * (cmd_right-cmd_left)*0.5
        omega_rob = cl * (cmd_right-cmd_left) * 0.5 + tr * (cmd_right+cmd_left) * 0.5
        y_dot_rob = omega_rob * d  # The model currently also estimates the offset of the camera position

        return [x_dot_rob,  y_dot_rob, omega_rob]

    @staticmethod
    def loadExperimentData():
        f = open(EXPERIMENT_NAME_FOR_PICKLE, 'rb')
        experimentDataObj = pickle.load(f)
        f.close()

        x_ramp_meas = experimentDataObj.x_ramp_meas
        y_ramp_meas = experimentDataObj.y_ramp_meas
        yaw_ramp_meas = experimentDataObj.yaw_ramp_meas

        x_sine_meas = experimentDataObj.x_sine_meas
        y_sine_meas = experimentDataObj.y_sine_meas
        yaw_sine_meas = experimentDataObj.yaw_sine_meas

        timepoints_ramp = experimentDataObj.timepoints_ramp
        time_ramp = experimentDataObj.time_ramp

        cmd_ramp_right = experimentDataObj.cmd_ramp_right
        cmd_ramp_left = experimentDataObj.cmd_ramp_left

        timepoints_sine = experimentDataObj.timepoints_sine
        time_sine = experimentDataObj.time_sine

        cmd_sine_right = experimentDataObj.cmd_sine_right
        cmd_sine_left = experimentDataObj.cmd_sine_left

        return (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
                x_sine_meas,y_sine_meas,yaw_sine_meas,
                timepoints_ramp, time_ramp ,
                cmd_ramp_right, cmd_ramp_left,
                timepoints_sine, time_sine,
                cmd_sine_right, cmd_sine_left)

    @staticmethod
    def resampling(time_ramp_meas, time_ramp_cmd, cmd_ramp_right, cmd_ramp_left, time_sine_meas, time_sine_cmd, cmd_sine_right, cmd_sine_left):
        # Sampling Time of the Identification
        Ts = 1.0/30
        # generate an equally spaced time vector over the full length of position measurement times
        # Based on looking at the data we assume the images were taken at a perfect
        # 30 FPS rate, but arrive in ROS at an variable rate. Therefore, we resample and
        # assume that the images have been taken at the ideal time steps
        time_ramp = np.arange(0,time_ramp_meas[-1]-time_ramp_meas[0]+Ts/2,Ts)
        time_sine = np.arange(0,time_sine_meas[-1]-time_sine_meas[0]+Ts/2,Ts)
        # Shift the cmd time vectors to match the new time vectors
        time_ramp_cmd = time_ramp_cmd - time_ramp_meas[0]
        time_sine_cmd = time_sine_cmd - time_sine_meas[0]
        # Resample commands to the fixed sampling time using the last command
        cmd_ramp_right = cmd_ramp_right[np.searchsorted(time_ramp_cmd, time_ramp, side='right')]
        cmd_ramp_left  = cmd_ramp_left[np.searchsorted(time_ramp_cmd, time_ramp, side='right')]
        cmd_sine_right = cmd_sine_right[np.searchsorted(time_sine_cmd, time_sine, side='right')]
        cmd_sine_left  = cmd_sine_left[np.searchsorted(time_sine_cmd, time_sine, side='right')]
        # timepoints (indexes in time vector) where we have measurements
        timepoints_ramp = ((time_ramp_meas-time_ramp_meas[0])*30+0.5).astype(int)
        timepoints_sine = ((time_sine_meas-time_sine_meas[0])*30+0.5).astype(int)

        return timepoints_ramp, time_ramp, cmd_ramp_right, cmd_ramp_left, timepoints_sine, time_sine, cmd_sine_right, cmd_sine_left

    def processData(self,starting_ind, ending_ind):
        (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
        x_sine_meas,y_sine_meas,yaw_sine_meas,
        timepoints_ramp,time_ramp ,
        cmd_ramp_right, cmd_ramp_left,
        timepoints_sine,time_sine,
        cmd_sine_right, cmd_sine_left) = self.loadExperimentData()

        x_ramp_meas = x_ramp_meas[starting_ind:ending_ind]
        y_ramp_meas = y_ramp_meas[starting_ind:ending_ind]
        yaw_ramp_meas = yaw_ramp_meas[starting_ind:ending_ind]

        x_sine_meas = x_sine_meas[starting_ind:ending_ind]
        y_sine_meas = y_sine_meas[starting_ind:ending_ind]
        yaw_sine_meas = yaw_sine_meas[starting_ind:ending_ind]

        timepoints_ramp = timepoints_ramp[starting_ind:ending_ind]
        time_ramp = time_ramp[starting_ind:ending_ind]

        cmd_ramp_right = cmd_ramp_right[starting_ind:ending_ind]
        cmd_ramp_left = cmd_ramp_left[starting_ind:ending_ind]

        timepoints_sine = timepoints_sine[starting_ind:ending_ind]
        time_sine = time_sine[starting_ind:ending_ind]

        cmd_sine_right = cmd_sine_right[starting_ind:ending_ind]
        cmd_sine_left = cmd_sine_left[starting_ind:ending_ind]

        return (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
                x_sine_meas,y_sine_meas,yaw_sine_meas,
                timepoints_ramp, time_ramp ,
                cmd_ramp_right, cmd_ramp_left,
                timepoints_sine, time_sine,
                cmd_sine_right, cmd_sine_left)

    def forwardEuler(self,s_cur, Ts, cmd_right, cmd_left,p):
        c, cl, tr = p

        x_0 = s_cur[0]
        y_0 = s_cur[1]
        yaw_0 = s_cur[2]

        vx_pred = c * (cmd_right+cmd_left) * 0.5 # + tr * (cmd_right-cmd_left)*0.5
        omega_pred = cl * (cmd_right-cmd_left) * 0.5 + tr * (cmd_right+cmd_left) * 0.5

        # np.cumsum looks unnecessary since we resampled
        yaw_pred = (omega_pred) * Ts + yaw_0
        x_pred = (np.cos(yaw_0) * vx_pred) * Ts + x_0
        y_pred = (np.sin(yaw_0) * vx_pred) * Ts + y_0

        """
        a = np.array([x_pred, yaw_pred, y_pred]).reshape(3)
        print("Shape of input a: {}".format(a.shape))
        print("Type of input a: {}".format(type(a)))
        """
        return np.array([x_pred, y_pred, yaw_pred]).reshape(3)

    def simulate(self,p, cmd_right, cmd_left, s_init,time):
        # States
        ## Note that values take checkerboard as the origin.
        s = np.zeros((len(time),3))

        s[0,0] = s_init[0] # x
        s[0,1] = s_init[1] # y
        s[0,2] = s_init[2] # yaw

        Ts = 1 / 30.0
        s_cur = s[0]

        #print("Shape of input s_cur: {}".format(s_cur.shape))
        #print("Type of input s_cur: {}".format(type(s_cur)))
        #print("Content time: {}".format(time))

        for i in range(len(time)-1):
            #ts = [time[i] * Ts, time[i+1]* Ts]
            """
            s_next = odeint(self.kinematic_model, s_cur, Ts, args=(cmd_right[i],cmd_left[i],p))
            s_cur = s_next[-1] # variable assignment
            """
            s_next = self.forwardEuler(s_cur, Ts, cmd_right[i],cmd_left[i],p)
            s_cur = np.copy(s_next)
            #print("Shape of input s_next: {}".format(s_next.shape))
            #print("Type of input s_next: {}".format(type(s_next)))
            #print("Content s_next: {}".format(s_next))

            #print("Shape of input s_cur: {}".format(s_cur.shape))
            #print("Type of input s_cur: {}".format(type(s_cur)))
            #print("Content s_next: {}".format(s_cur))
            s[i+1] = s_cur

        return s

    def objective_ramp(self,p, cmd_right, cmd_left, s_init, x_meas, y_meas, yaw_meas, time):
        self.OBJECTIVE_FN_COUNTER += 1
        #print "OBJECTIVE FN CALL NUMBER: {}".format(self.OBJECTIVE_FN_COUNTER)

        #simulate the model
        #states for particular p set
        s_p = self.simulate(p, cmd_right, cmd_left, s_init, time)

        obj_cost = 0.0

        for i in range(len(time)):
            """
            obj_cost+= ( ((s_p[i,0] - x_meas[i])/ x_meas[i]) ** 2 +
                         ((s_p[i,1] - y_meas[i])/ y_meas[i]) ** 2 +
                         ((s_p[i,2] - yaw_meas[i])/ yaw_meas[i]) ** 2
                       )
            """
            obj_cost+= ( ((s_p[i,0] - x_meas[i])/ x_meas[i]) ** 2)
            #print "iter: {} obj value: {} ".format(i, obj_cost)
        # calculate the objective cost
        return obj_cost
    def objective_sine(self,p, cmd_right, cmd_left, s_init, x_meas, y_meas, yaw_meas, time):
        self.OBJECTIVE_FN_COUNTER += 1
        #print "OBJECTIVE FN CALL NUMBER: {}".format(self.OBJECTIVE_FN_COUNTER)

        #simulate the model
        #states for particular p set
        s_p = self.simulate(p, cmd_right, cmd_left, s_init, time)

        obj_cost = 0.0

        for i in range(len(time)):

            obj_cost+= ( 10 * ((s_p[i,0] - x_meas[i])/ x_meas[i]) ** 2 +
                         ((s_p[i,1] - y_meas[i])/ y_meas[i]) ** 2 +
                         ((s_p[i,2] - yaw_meas[i])/ yaw_meas[i]) ** 2
                       )

            #obj_cost+= ( ((s_p[i,0] - x_meas[i])/ x_meas[i]) ** 2)
            #print "iter: {} obj value: {} ".format(i, obj_cost)
        # calculate the objective cost
        return obj_cost
    def objective_combined(self,p, cmd_right_ramp, cmd_left_ramp, s_init_ramp, x_meas_ramp, y_meas_ramp, yaw_meas_ramp, time_ramp, cmd_right_sine, cmd_left_sine, s_init_sine, x_meas_sine, y_meas_sine, yaw_meas_sine, time_sine):
        #self.OBJECTIVE_FN_COUNTER += 1
        #print "OBJECTIVE FN CALL NUMBER: {}".format(self.OBJECTIVE_FN_COUNTER)

        #simulate the model
        #states for a particular p set
        s_p_ramp = self.simulate(p, cmd_right_ramp, cmd_left_ramp, s_init_ramp, time_ramp)
        s_p_sine = self.simulate(p, cmd_right_sine, cmd_left_sine, s_init_sine, time_sine)

        obj_cost = 0.0
        """
        for i in range(len(time_ramp)):
            obj_cost+= (((s_p_ramp[i,0] - x_meas_ramp[i])/ x_meas_ramp[i]) ** 2 +
                        ((s_p_ramp[i,1] - y_meas_ramp[i])/ y_meas_ramp[i]) ** 2 +
                        ((s_p_ramp[i,2] - yaw_meas_ramp[i])/ yaw_meas_ramp[i]) ** 2
                       )

        for i in range(len(time_sine)):
            obj_cost+= (((s_p_sine[i,0] - x_meas_sine[i])/ x_meas_sine[i]) ** 2 +
                        ((s_p_sine[i,1] - y_meas_sine[i])/ y_meas_sine[i]) ** 2 +
                        ((s_p_sine[i,2] - yaw_meas_sine[i])/ yaw_meas_sine[i]) ** 2
                       )
        """
        for i in range(len(time_ramp)):
            obj_cost+= (((s_p_ramp[i,0] - x_meas_ramp[i])) ** 2 +
                        ((s_p_ramp[i,1] - y_meas_ramp[i])) ** 2 +
                        ((s_p_ramp[i,2] - yaw_meas_ramp[i])) ** 2
                       )

        for i in range(len(time_sine)):
            obj_cost+= (((s_p_sine[i,1] - y_meas_sine[i])) ** 2 +
                        ((s_p_sine[i,2] - yaw_meas_sine[i])) ** 2 +
                        ((s_p_sine[i,2] - yaw_meas_sine[i])) ** 2
                       )

            #obj_cost+= ( ((s_p[i,0] - x_meas[i])/ x_meas[i]) ** 2)
            #print "iter: {} obj value: {} ".format(i, obj_cost)
        # calculate the objective cost
        return obj_cost

    def experiment_ramp(self):
        print ("\nBEG: EXPERIMENT_RAMP FN")
        # select portion of data to use
        start = 0
        end = -10

        (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
        x_sine_meas,y_sine_meas,yaw_sine_meas,
        timepoints_ramp,time_ramp ,
        cmd_ramp_right, cmd_ramp_left,
        timepoints_sine,time_sine,
        cmd_sine_right, cmd_sine_left) = self.processData(starting_ind = start, ending_ind = end)
        #######################################################################################################################
        ### IDENTIFICATION FOR RAMP MANOUVER ###
        # assign the initial
        #initial conditions for the states
        s_init_ramp = [x_ramp_meas[start], y_ramp_meas[start], yaw_ramp_meas[start]]
        #initial guesses for the parameters: c, cl, tr
        #p0 = [0.6, 6, 0]
        p0 = [0.6, 6.0, 0.0]

        y_pred_ramp_default_params = self.simulate(p0, cmd_ramp_right, cmd_ramp_left, s_init_ramp , timepoints_ramp)
        self.y_pred_ramp_default_params = y_pred_ramp_default_params

        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model predition
        result_ramp = minimize(self.objective_ramp, p0, args=(cmd_ramp_right, cmd_ramp_left, s_init_ramp, x_ramp_meas,  y_ramp_meas, yaw_ramp_meas, timepoints_ramp))
        popt_ramp = result_ramp.x

        print "XXXXXXXXXXXXXXXXXXXXXXXXXX"
        print(type(popt_ramp))
        print popt_ramp
        print "XXXXXXXXXXXXXXXXXXXXXXXXXX"

        print result_ramp

        # Make a prediction based on the fitted parameters
        y_opt_predict_ramp = self.simulate(popt_ramp, cmd_ramp_right, cmd_ramp_left, s_init_ramp , timepoints_ramp) # Predict to calculate Error
        self.y_opt_predict_ramp = y_opt_predict_ramp
        Y = np.stack((x_ramp_meas, y_ramp_meas,yaw_ramp_meas), axis=1)
        MSE_ramp = np.sum((Y-y_opt_predict_ramp)**2)/y_opt_predict_ramp.size # Calculate the Mean Squared Error

        # PLOTTING
        fig2, ax1 = plt.subplots()

        x_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],x_ramp_meas,'x',color=(0.5,0.5,1), label = 'x measured')
        y_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],y_ramp_meas,'x',color=(0.5,1,0.5), label = 'y measured')
        yaw_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],yaw_ramp_meas,'x',color=(1,0.5,0.5), label = 'yaw measured')
        # Model predictions with default parameters
        x_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,0],'bo', label = 'x predict def')
        y_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,2],'go', label = 'y predict def')
        yaw_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,1],'ro', label = 'yaw predict def')


        # Model predictions with optimal parametes
        x_ramp_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_ramp[:,0],'b', label = 'x predict opt')
        y_ramp_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_ramp[:,1],'g', label = 'y predict opt')
        yaw_ramp_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_ramp[:,2],'r', label = 'yaw predict opt')

        ax1.set_xlabel('time [s]')
        ax1.set_ylabel('position [m] / heading [rad]')

        """
        ax2 = ax1.twinx()
        cmd_ramp_right_handle, = ax2.plot(time_ramp[timepoints_ramp],cmd_ramp_right,'bx', label = 'right motor')
        cmd_ramp_left_handle, =ax2.plot(time_ramp[timepoints_ramp],cmd_ramp_left, 'r+', label = 'left motor')
        ax2.set_ylabel('PWM [%]')

        """

        """
        handles = [x_ramp_meas_handle,y_ramp_meas_handle,yaw_ramp_meas_handle,
                   x_ramp_pred_default_handle, y_ramp_pred_default_handle, yaw_ramp_pred_default_handle]
        """
        handles = [x_ramp_meas_handle,y_ramp_meas_handle,yaw_ramp_meas_handle,
                   x_ramp_pred_handle,y_ramp_pred_handle,yaw_ramp_pred_handle,
                   x_ramp_pred_default_handle, y_ramp_pred_default_handle, yaw_ramp_pred_default_handle]
        #,cmd_ramp_right_handle,cmd_ramp_left_handle

        labels = [h.get_label() for h in handles]

        fig2.legend(handles=handles, labels=labels, prop={'size': 10}, loc=0)
        fig2.suptitle('Measurements and Prediction with Default/Optimal Parameter Values - Ramp Manouver', fontsize=16)
        plt.show(block=True)
        """
        print("Size of input cmd_ramp_right: {}".format(cmd_ramp_right.shape))
        print("Size of input cmd_ramp_left: {}".format(cmd_ramp_left.shape))
        print("Size of input x_ramp_meas: {}".format(x_ramp_meas.shape))
        #print("Content x_ramp_meas: {}".format(x_ramp_meas))
        print("Size of input y_ramp_meas: {}".format(y_ramp_meas.shape))
        #print("Content y_ramp_meas: {}".format(y_ramp_meas))
        print("Size of input yaw_ramp_meas: {}".format(yaw_ramp_meas.shape))
        #print("Content yaw_ramp_meas: {}".format(yaw_ramp_meas))
        print("Size of input timepoints_ramp: {}".format(timepoints_ramp.shape))
        print("Size of input time_ramp: {}".format(time_ramp.shape))
        """
        """
        print('Optimization Result for Ramp Manouver\n')
        print(result_ramp)
        print('Optimization Result for Ramp Manouver\n')


        print("Shape of input y_opt_predict_ramp: {}".format(y_opt_predict_ramp.shape))
        #print("Type of input y_opt_predict_ramp: {}".format(type(y_opt_predict_ramp)))
        #print("Content y_opt_predict_ramp: {}".format(y_opt_predict_ramp))
        """

        """
        print("Shape of input x_ramp_meas: {}".format(x_ramp_meas.shape))
        print("Shape of input y_ramp_meas: {}".format(y_ramp_meas.shape))
        print("Shape of input yaw_ramp_meas: {}".format(yaw_ramp_meas.shape))
        print("Shape of input Y : {}".format(Y.shape))
        """


        """
        ### START: PLOT MOTOR INPUT SEQUENCE vs TIME
        plt.figure(1)
        plt.plot(time_ramp,cmd_ramp_right, 'bx')
        plt.plot(time_ramp,cmd_ramp_left, 'r+')

        plt.legend(['right motor','left motor'],loc=4)
        plt.title('Input Sequence - Ramp Manouver')
        plt.xlabel('time [s]')
        plt.ylabel('PWM [%]')
        plt.show(block=False)
        #### END: PLOT MOTOR INPUT SEQUENCE vs TIME
        """

        print ("\nEND: EXPERIMENT_RAMP FN")
        return popt_ramp
    def experiment_sine(self):
        print ("\nBEG: EXPERIMENT_SINE FN")
        # select portion of data to use
        start = 0
        end = -10

        (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
        x_sine_meas,y_sine_meas,yaw_sine_meas,
        timepoints_ramp,time_ramp ,
        cmd_ramp_right, cmd_ramp_left,
        timepoints_sine,time_sine,
        cmd_sine_right, cmd_sine_left) = self.processData(starting_ind = start, ending_ind = end)
        #######################################################################################################################
        ### IDENTIFICATION FOR RAMP MANOUVER ###
        # assign the initial
        #initial conditions for the states
        s_init_sine = [x_sine_meas[start], y_sine_meas[start], yaw_sine_meas[start]]
        #initial guesses for the parameters: c, cl, tr
        #p0 = [0.6, 6, 0]
        p0 = [0.6, 6, 0]
        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model predition
        result_sine = minimize(self.objective_sine, p0, args=(cmd_sine_right, cmd_sine_left, s_init_sine, x_sine_meas,  y_sine_meas, yaw_sine_meas, timepoints_sine))
        popt_sine = result_sine.x

        # Make a prediction based on the fitted parameters
        y_opt_predict_sine = self.simulate(popt_sine, cmd_sine_right, cmd_sine_left, s_init_sine , timepoints_sine) # Predict to calculate Error
        self.y_opt_predict_sine = y_opt_predict_sine

        #print("Type of input y_opt_predict_sine: {}".format(type(y_opt_predict_sine)))
        #print("Content y_opt_predict_sine: {}".format(y_opt_predict_sine))

        Y = np.stack((x_sine_meas, y_sine_meas, yaw_sine_meas), axis=1)

        MSE_sine = np.sum((Y-y_opt_predict_sine)**2)/y_opt_predict_sine.size # Calculate the Mean Squared Error

        y_pred_sine_default_params = self.simulate(p0, cmd_sine_right, cmd_sine_left, s_init_sine , timepoints_sine)
        self.y_pred_sine_default_params = y_pred_sine_default_params
        y_pred_sine_default_params = y_pred_sine_default_params.reshape((int(y_pred_sine_default_params.size/3),3),order='F')

        # PLOTTING
        fig2, ax1 = plt.subplots()

        x_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],x_sine_meas,'x',color=(0.5,0.5,1), label = 'x measured')
        y_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],y_sine_meas,'x',color=(0.5,1,0.5), label = 'y measured')
        yaw_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],yaw_sine_meas,'x',color=(1,0.5,0.5), label = 'yaw measured')
        # Model predictions with default parameters
        x_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,0],'bo', label = 'x predict def')
        y_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,2],'go', label = 'y predict def')
        yaw_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,1],'ro', label = 'yaw predict def')


        # Model predictions with optimal parametes
        x_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,0],'b', label = 'x predict opt')
        y_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,1],'g', label = 'y predict opt')
        yaw_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,2],'r', label = 'yaw predict opt')

        ax1.set_xlabel('time [s]')
        ax1.set_ylabel('position [m] / heading [rad]')

        """
        ax2 = ax1.twinx()
        cmd_sine_right_handle, = ax2.plot(time_sine[timepoints_sine],cmd_sine_right,'bx', label = 'right motor')
        cmd_sine_left_handle, =ax2.plot(time_sine[timepoints_sine],cmd_sine_left, 'r+', label = 'left motor')
        ax2.set_ylabel('PWM [%]')
        """

        """
        handles = [x_sine_meas_handle,y_sine_meas_handle,yaw_sine_meas_handle,
                   x_sine_pred_default_handle, y_sine_pred_default_handle, yaw_sine_pred_default_handle]
        """
        handles = [x_sine_meas_handle,y_sine_meas_handle,yaw_sine_meas_handle,
                   x_sine_pred_handle,y_sine_pred_handle,yaw_sine_pred_handle,
                   x_sine_pred_default_handle, y_sine_pred_default_handle, yaw_sine_pred_default_handle]
        #,cmd_sine_right_handle,cmd_sine_left_handle

        labels = [h.get_label() for h in handles]

        fig2.legend(handles=handles, labels=labels, prop={'size': 10}, loc=0)
        fig2.suptitle('Measurements and Prediction with Default/Optimal Parameter Values - Sine Manouver', fontsize=16)
        plt.show(block=True)

        """
        print("Shape of input cmd_sine_right: {}".format(cmd_sine_right.shape))
        print("Shape of input cmd_sine_left: {}".format(cmd_sine_left.shape))
        print("Shape of input x_sine_meas: {}".format(x_sine_meas.shape))
        #print("Content x_sine_meas: {}".format(x_sine_meas))
        print("Shape of input y_sine_meas: {}".format(y_sine_meas.shape))
        #print("Content y_sine_meas: {}".format(y_sine_meas))
        print("Shape of input yaw_sine_meas: {}".format(yaw_sine_meas.shape))
        #print("Content yaw_sine_meas: {}".format(yaw_sine_meas))
        print("Shape of input timepoints_sine: {}".format(timepoints_sine.shape))
        print("Shape of input time_sine: {}".format(time_sine.shape))
        print("Shape of input y_opt_predict_sine: {}".format(y_opt_predict_sine.shape))
        print("Shape of input y_pred_sine_default_params: {}".format(y_pred_sine_default_params.shape))
        """

        print('[BEGIN] Optimization Result for sine Manouver\n')
        print(result_sine)
        print('[END] Optimization Result for sine Manouver\n')


        print ("\nEND: EXPERIMENT_SINE FN")
        return popt_sine
    def experiment_combined(self):
        print ("\nBEG: EXPERIMENT_SINE FN")
        # select portion of data to use
        start = 0
        end = -10

        (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
        x_sine_meas,y_sine_meas,yaw_sine_meas,
        timepoints_ramp,time_ramp ,
        cmd_ramp_right, cmd_ramp_left,
        timepoints_sine,time_sine,
        cmd_sine_right, cmd_sine_left) = self.processData(starting_ind = start, ending_ind = end)
        ###########################################################################################################
        #initial conditions for the states
        s_init_sine = [x_sine_meas[start], y_sine_meas[start], yaw_sine_meas[start]]
        s_init_ramp = [x_ramp_meas[start], y_ramp_meas[start], yaw_ramp_meas[start]]
        #initial guesses for the parameters: c, cl, tr
        #p0 = [0.6, 6, 0]
        p0 = [0.6, 6, 0]
        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model predition
        result_combined = minimize(self.objective_combined, p0, args=(cmd_ramp_right, cmd_ramp_left, s_init_ramp, x_ramp_meas, y_ramp_meas, yaw_ramp_meas, timepoints_ramp, cmd_sine_right, cmd_sine_left, s_init_sine, x_sine_meas, y_sine_meas, yaw_sine_meas, timepoints_sine))
        popt_combined = result_combined.x

        # Make a prediction based on the fitted parameters
        y_opt_predict_combine = self.simulate(popt_combined, cmd_sine_right, cmd_sine_left, s_init_sine , timepoints_sine) # Predict to calculate Error
        self.y_opt_predict_combine = y_opt_predict_combine

        #print("Type of input y_opt_predict_combine: {}".format(type(y_opt_predict_combine)))
        #print("Content y_opt_predict_combine: {}".format(y_opt_predict_combine))

        Y = np.stack((x_sine_meas, y_sine_meas, yaw_sine_meas), axis=1)

        MSE_sine = np.sum((Y-y_opt_predict_combine)**2)/y_opt_predict_combine.size # Calculate the Mean Squared Error

        y_pred_sine_default_params = self.simulate(p0, cmd_sine_right, cmd_sine_left, s_init_sine , timepoints_sine)
        self.y_pred_sine_default_params = y_pred_sine_default_params
        y_pred_sine_default_params = y_pred_sine_default_params.reshape((int(y_pred_sine_default_params.size/3),3),order='F')

        # PLOTTING
        fig2, ax1 = plt.subplots()

        x_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],x_sine_meas,'x',color=(0.5,0.5,1), label = 'x measured')
        y_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],y_sine_meas,'x',color=(0.5,1,0.5), label = 'y measured')
        yaw_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],yaw_sine_meas,'x',color=(1,0.5,0.5), label = 'yaw measured')
        # Model predictions with default parameters
        x_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,0],'bo', label = 'x predict def')
        y_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,2],'go', label = 'y predict def')
        yaw_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,1],'ro', label = 'yaw predict def')


        # Model predictions with optimal parametes
        x_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_combine[:,0],'b', label = 'x predict opt')
        y_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_combine[:,1],'g', label = 'y predict opt')
        yaw_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_combine[:,2],'r', label = 'yaw predict opt')

        ax1.set_xlabel('time [s]')
        ax1.set_ylabel('position [m] / heading [rad]')

        """
        ax2 = ax1.twinx()
        cmd_sine_right_handle, = ax2.plot(time_sine[timepoints_sine],cmd_sine_right,'bx', label = 'right motor')
        cmd_sine_left_handle, =ax2.plot(time_sine[timepoints_sine],cmd_sine_left, 'r+', label = 'left motor')
        ax2.set_ylabel('PWM [%]')
        """

        """
        handles = [x_sine_meas_handle,y_sine_meas_handle,yaw_sine_meas_handle,
                   x_sine_pred_default_handle, y_sine_pred_default_handle, yaw_sine_pred_default_handle]
        """
        handles = [x_sine_meas_handle,y_sine_meas_handle,yaw_sine_meas_handle,
                   x_sine_pred_handle,y_sine_pred_handle,yaw_sine_pred_handle,
                   x_sine_pred_default_handle, y_sine_pred_default_handle, yaw_sine_pred_default_handle]
        #,cmd_sine_right_handle,cmd_sine_left_handle

        labels = [h.get_label() for h in handles]

        fig2.legend(handles=handles, labels=labels, prop={'size': 10}, loc=0)
        fig2.suptitle('Measurements and Prediction with Default/Optimal(Combined CF) Parameter Values - Sine Manouver', fontsize=16)
        plt.show(block=True)

        print('[BEGIN] Optimization Result for sine Manouver\n')
        print(result_combined)
        print('[END] Optimization Result for sine Manouver\n')


        print ("\nEND: EXPERIMENT_SINE FN")
        return popt_combined

    def nonlinear_model_fit(self):
        if CALIBRATION_FORMULATION_APPROACH == "seperate":
            popt_ramp = self.experiment_ramp()
            popt_sine = self.experiment_sine()
            #fit ={'c':popt_ramp[0],'cl':popt_sine[1],'tr':popt_ramp[2]}

            popt_mixed = np.array([popt_ramp[0], popt_sine[1], popt_ramp[2]]).reshape(3)
            fit ={'c':popt_ramp[0],'cl':popt_sine[1],'tr':popt_ramp[2]}

            # select portion of data to use
            start = 0
            end = -10

            (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
            x_sine_meas,y_sine_meas,yaw_sine_meas,
            timepoints_ramp,time_ramp ,
            cmd_ramp_right, cmd_ramp_left,
            timepoints_sine,time_sine,
            cmd_sine_right, cmd_sine_left) = self.processData(starting_ind = start, ending_ind = end)

            s_init_ramp = [x_ramp_meas[start], y_ramp_meas[start], yaw_ramp_meas[start]]
            #initial guesses for the parameters: c, cl, tr
            #p0 = [0.6, 6, 0]
            p0 = [0.6, 6.0, 0.0]

            # Make a prediction based on the fitted parameters
            y_opt_predict_mixed = self.simulate(popt_mixed, cmd_ramp_right, cmd_ramp_left, s_init_ramp , timepoints_ramp) # Predict to calculate Error

            Y = np.stack((x_ramp_meas, y_ramp_meas,yaw_ramp_meas), axis=1)
            MSE_ramp = np.sum((Y-y_opt_predict_mixed)**2)/y_opt_predict_mixed.size # Calculate the Mean Squared Error

            y_pred_ramp_default_params = self.y_pred_ramp_default_params

            # PLOTTING
            fig2, ax1 = plt.subplots()

            x_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],x_ramp_meas,'x',color=(0.5,0.5,1), label = 'x measured')
            y_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],y_ramp_meas,'x',color=(0.5,1,0.5), label = 'y measured')
            yaw_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],yaw_ramp_meas,'x',color=(1,0.5,0.5), label = 'yaw measured')
            # Model predictions with default parameters
            x_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,0],'bo', label = 'x predict def')
            y_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,2],'go', label = 'y predict def')
            yaw_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,1],'ro', label = 'yaw predict def')


            # Model predictions with optimal parametes
            x_mixed_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_mixed[:,0],'b', label = 'x predict opt')
            y_mixed_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_mixed[:,1],'g', label = 'y predict opt')
            yaw_mixed_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_mixed[:,2],'r', label = 'yaw predict opt')

            ax1.set_xlabel('time [s]')
            ax1.set_ylabel('position [m] / heading [rad]')

            handles = [x_ramp_meas_handle,y_ramp_meas_handle,yaw_ramp_meas_handle,
                       x_mixed_pred_handle,y_mixed_pred_handle,yaw_mixed_pred_handle,
                       x_ramp_pred_default_handle, y_ramp_pred_default_handle, yaw_ramp_pred_default_handle]
            #,cmd_ramp_right_handle,cmd_ramp_left_handle

            labels = [h.get_label() for h in handles]

            fig2.legend(handles=handles, labels=labels, prop={'size': 10}, loc=0)
            fig2.suptitle('Measurements and Prediction with Default/Combined-Optimal Parameter Values - Ramp Manouver', fontsize=16)
            plt.show(block=True)

        elif CALIBRATION_FORMULATION_APPROACH == "combined":
            popt_combined = self.experiment_combined()
            fit ={'c':popt_combined[0],'cl':popt_combined[1],'tr':popt_combined[2]}

        return fit

    def write_calibration(self):
        '''Load kinematic calibration file'''
        filename = (get_duckiefleet_root() + "/calibrations/kinematics/" + self.robot_name + ".yaml")
        if not os.path.isfile(filename):
            logger.warn("no kinematic calibration parameters for {}, taking some from default".format(self.robot_name))
            filename = (get_duckiefleet_root() + "/calibrations/kinematics/default.yaml")
            if not os.path.isfile(filename):
                logger.error("can't find default either, something's wrong, is the duckiefleet root correctly set?")
            else:
                data = yaml_load_file(filename)
        else:
            rospy.loginfo("Loading some kinematic calibration parameters of " + self.robot_name)
            data = yaml_load_file(filename)
        logger.info("Loaded homography for {}".format(os.path.basename(filename)))

        # Load some of the parameters that will not be changed
        param_k        = data['k']
        param_limit    = data['limit']
        param_radius   = data['radius']

        # simply to increase readability
        c  = self.fit_['c']
        cl = self.fit_['cl']
        tr = self.fit_['tr']

        # Calculation of Kinematic Calibration parameters from our model parameters
        # Due to the redundancy of the k and radius parameter, the read parameters are taken into account
        # We chose to overwrite the gain parameter, but instead the motor constant could be changed
        gain = param_radius * param_k / c
        trim = - tr * param_radius * param_k * param_radius * param_k / (cl * c)
        baseline = 2 * c / cl * 0.9  # The 0.9 is a biasing factor because the sine steer is finite

        # Write to yaml
        #datasave = {  # This is similar to the inverse_kinematics_node, but it did not work...
        #    "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
        #    "gain": gain,
        #    "trim": trim,
        #    "baseline": baseline,
        #    "radius": param_radius,
        #    "k": param_k,
        #    "limit": param_limit,
        #}
        datasave = \
        "calibration_time: {}".format(time.strftime("%Y-%m-%d-%H-%M-%S")) + \
        "\ngain: {}".format(gain) + \
        "\ntrim: {}".format(trim) + \
        "\nbaseline: {}".format(baseline) + \
        "\nradius: {}".format(param_radius) + \
        "\nk: {}".format(param_k) + \
        "\nlimit: {}".format(param_limit)


        print("\nThe Estimated Kinematic Calibration is:")
        print("gain     = {}".format(gain))
        print("trim     = {}".format(trim))
        print("baseline = {}".format(baseline))

        # Write to yaml file
        filename = (get_duckiefleet_root() + "/calibrations/kinematics/" + self.robot_name + ".yaml")
        with open(filename, 'w') as outfile:
            outfile.write(datasave)
            #outfile.write(yaml.dump(datasave, default_flow_style=False))  # This did not work and gave very weird results

        print("Saved Parameters to " + self.robot_name + ".yaml" )

        print("\nPlease check the plots and judge if the parameters are reasonable.")
        print("Once done inspecting the plot, close them to terminate the program.")

    def plots(self):
        print ("BEG: PLOTS FN")
        y_pred_ramp_default_params = self.y_pred_ramp_default_params
        y_opt_predict_ramp = self.y_opt_predict_ramp
        y_pred_sine_default_params = self.y_pred_sine_default_params
        y_opt_predict_sine = self.y_opt_predict_sine

        # select portion of data to use
        start = 0
        end = -10

        (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
        x_sine_meas,y_sine_meas,yaw_sine_meas,
        timepoints_ramp,time_ramp ,
        cmd_ramp_right, cmd_ramp_left,
        timepoints_sine,time_sine,
        cmd_sine_right, cmd_sine_left) = self.processData(starting_ind = start, ending_ind = end)

        plt.figure(3)
        plt.plot(time_sine[timepoints_sine],x_sine_meas,'x',color=(0.5,0.5,1))
        plt.plot(time_sine[timepoints_sine],y_sine_meas,'x',color=(0.5,1,0.5))
        plt.plot(time_sine[timepoints_sine],yaw_sine_meas,'x',color=(1,0.5,0.5))

        print("Size of input y_opt_predict_sine: {}".format(y_opt_predict_sine[:,0].shape))
        print("Size of input time_sine: {}".format(time_sine.shape))


        plt.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,0],'b')
        plt.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,1],'g')
        plt.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,2],'r')

        plt.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,0],'bo')
        plt.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,1],'go')
        plt.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,2],'ro')

        plt.legend(['x measured','y measured','yaw measured','x predict opt','y predict opt','yaw predict opt','x predict default','y predict default','yaw predict default'],loc=4)
        plt.title('Measurements and Prediction with Default/Optimal Parameter Values - Sine Manouver')
        plt.xlabel('time [s]')
        plt.ylabel('position [m] / heading [rad]')
        plt.show(block=True)

        print ("END: PLOTS FN")

    def visualize(self):  # For Debugging only
        Ts = 1.0 / 30
        time = np.arange(0, Ts * self.veh_pose_['straight']['z_veh'].size, Ts)
        # PLOTTING
        plt.figure(1)
        plt.plot(time, self.veh_pose_['straight']['z_veh'],'r')
        plt.plot(time, self.veh_pose_['straight']['x_veh'], 'b')
        plt.plot(time, self.veh_pose_['straight']['y_veh'], 'y')
        plt.plot(time, self.veh_pose_['straight']['pitch_veh'], 'b--')
        plt.plot(time, self.veh_pose_['straight']['roll_veh'], 'y--')
        plt.plot(time, self.veh_pose_['straight']['yaw_veh'], 'r--')

        plt.legend(['z measured', 'x measured', 'y measured', 'pitch measured', 'roll measured', 'yaw measured', 'yaw predicted'], loc=4)
        plt.title('Experiment 1')
        plt.xlabel('time [s]')
        plt.ylabel('position [m] / heading [rad]')

        # Plot figure
        fig = plt.figure(2)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.veh_pose_['straight']['x_veh'], self.veh_pose_['straight']['y_veh'], self.veh_pose_['straight']['z_veh'])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.figure(3)
        plt.plot(self.veh_pose_['straight']['x_veh'], self.veh_pose_['straight']['y_veh'])


        plt.figure(4)
        time = np.arange(0, Ts * self.veh_pose_['curve']['z_veh'].size, Ts)
        plt.plot(time, self.veh_pose_['curve']['z_veh'],'r')
        plt.plot(time, self.veh_pose_['curve']['x_veh'], 'b')
        plt.plot(time, self.veh_pose_['curve']['y_veh'], 'y')
        plt.plot(time, self.veh_pose_['curve']['pitch_veh'], 'b--')
        plt.plot(time, self.veh_pose_['curve']['roll_veh'], 'y--')
        plt.plot(time, self.veh_pose_['curve']['yaw_veh'], 'r--')

        plt.legend(['z measured', 'x measured', 'y measured', 'pitch measured', 'roll measured', 'yaw measured', 'yaw predicted'], loc=4)
        plt.title('Experiment 2')
        plt.xlabel('time [s]')
        plt.ylabel('position [m] / heading [rad]')

        # Plot figure
        fig=plt.figure(5)
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.veh_pose_['curve']['x_veh'], self.veh_pose_['curve']['y_veh'], self.veh_pose_['curve']['z_veh'])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.figure(6)
        plt.plot(self.veh_pose_['curve']['x_veh'], self.veh_pose_['curve']['y_veh'])

        # show all figures
        plt.show()


####################
# Main part of the script

if __name__ == '__main__':
    calib=calib()
