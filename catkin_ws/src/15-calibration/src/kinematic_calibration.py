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
from mpl_toolkits.mplot3d import Axes3D
import yaml


class calib():
    def __init__(self):

	# Initialize the node with rospy
        rospy.init_node('command', anonymous=True)

        # defaults overwritten by param
        self.robot_name = rospy.get_param("~veh")

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


        # uncomment to try nonlinear_model
        self.fit_=self.nonlinear_model_fit()

        # write to the kinematic calibration file
        self.write_calibration()

        # make plots & visualizations
        # self.plot=self.visualize()
        plt.show()


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

    def draw(self,     img, corners, imgpts):
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


        chw=self.board_['width']
        chh=self.board_['height']
        patternSize = (chw, chh)
        chSeize=self.board_['square_size']
        # termination criteria
        # if resolution is low keep iteration to minimum otherwise it messes up the code
        # the process of corner refinement stops either after critera.max.count iteration or when the corner moves by less than criteria.epsilon on some iteration
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,20,0.01)  # befor was 30 instead of 20
        # find_chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_FILTER_QUADS
        find_chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chw * chh, 3), np.float32)
        objp[:, :2] = np.mgrid[0:chw, 0:chh].T.reshape(-1, 2)
        objp = objp.reshape(-1, 1, 3)
        objp = np.multiply(objp, chSeize)

        # Create the axis points
        axisPoints = np.float32([[3*chSeize, 0, 0], [0, 3*chSeize, 0], [0, 0, -3*chSeize]]).reshape(-1, 3)
        inputbag=rospy.get_param("~path")+self.robot_name+"_calibration.bag"
        topicname = "/" + self.robot_name + "/camera_node/image/compressed"
        indexcounter=1

        #initialize recording state
        Recording=False


        stopIndex = [i for i, j in enumerate(self.wheels_cmd_['vel_r']) if j == 0]
        startIndex = [i for i, j in enumerate(self.wheels_cmd_['vel_r']) if j != 0]
        stopTime=[self.wheels_cmd_['timestamp'][i] for i in stopIndex]
        startTime=[self.wheels_cmd_['timestamp'][i] for i in startIndex]
        counter=0
        
        #for i in range(0,np.size(self.wheels_cmd_['timestamp'])):
        #    self.wheels_cmd_['timestamp'][i] = self.wheels_cmd_['timestamp'][i].to_sec()

       

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
                    print "checkerboard in frame %d found" %indexcounter
                    corners2 = cv2.cornerSubPix(gray, corners, (5,5), (-1, -1), criteria)
                    
                    # only reverse order if first point is at bottom right corner
                    if corners2[0][0][0]>corners2[34][0][0]:
                        #print "Reversing order of points."
                        corners2=corners2[::-1]

                    # Draw and display the corners
                    img = cv2.drawChessboardCorners(img, (chw, chh), corners2, ret)

                    # Find the rotation and translation vectors.
                    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, self.cam.K, self.cam.D)

                    #Red Z-axis = pointing away from image
                    #Blue X-axis = pointing right or left
                    #Green Y-axis = pointing down or up


                    # OLD CALCULATION OF VEH POSITION
                    '''
                    # camera pose from object points
                    R_chess_cam = cv2.Rodrigues(rvecs)[0]
                    cam_pos = -np.matrix(R_chess_cam).T * np.matrix(tvecs) #camera pose from perspective of chessboard

                    #translate coordinate cam_pose to vehicle poseS
                    # angle_cam_veh = 0
                    angle_cam_veh=12/180.0*math.pi #guess of camera angle in degreee to veh around x-axis
                    R_cam_verticalcam=self.ZYXEuler2rot([float(angle_cam_veh),0.0,0.0])
                    # assert (self.isRotationMatrix(R_cam_verticalcam))

                    #change direction of coordinates system
                    R_verticalcam_veh= np.matrix('0 1 0; 0 0 1; 1 0 0')
                    # R_verticalcam_veh=np.eye(3)
                    #turn get eulerangles of R_chess_veh=R_chess_cam*R_cam_veh
                    # R_chess_veh = np.matmul(R_chess_cam,R_cam_verticalcam)
                    R_chess_veh = R_chess_cam.dot(R_cam_verticalcam).dot(R_verticalcam_veh)
                    # print R_chess_veh
                    veh_pos = np.matrix(R_chess_veh).T * np.matrix(tvecs) #camera pose from perspective of chessboard
                    # get euler angles [x,y,z] in radians
                    veh_eulerangles=self.rot2ZYXEuler(R_chess_veh)
                    '''



                    t_chess_cam=tvecs

                    # camera pose from object points
                    R_cam_chess = cv2.Rodrigues(rvecs)[0]
                    R_chess_cam=np.matrix(R_cam_chess).T
                    t_cam_chess =-np.matrix(R_cam_chess).T * np.matrix(tvecs)

                    T_chess_cam = np.zeros((4, 4))
                    T_chess_cam[3, 3] = 1
                    T_chess_cam[:3, :3] = R_chess_cam
                    # T_chess_cam[:3, :3] = R_cam_chess
                    T_chess_cam[:3, 3] = t_cam_chess.transpose()
                    # T_chess_cam[:3, 3] = t_chess_cam.transpose()

                    # change direction of coordinates system
                    R_world_chess=np.matrix('0 0 1;-1 0 0;0 -1 0')
                    # R_world_chess=np.matrix('1 0 0;0 0 1;0 -1 0')

                    # R_world_chess=np.eye(3)
                    T_world_chess = np.zeros((4, 4))
                    T_world_chess[3, 3] = 1
                    T_world_chess[:3, :3] = R_world_chess
                    T_world_chess[:3, 3] = [0, 0, 0]

                    #translate coordinate cam_pose to vehicle pose
                    angle =0/180.0*math.pi #guess of camera angle in degree to veh
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

        #reshape for pose for better handling
        # x = np.reshape(x, len(x))
        # y = np.reshape(y, len(x))
        # z = np.reshape(z, len(x))
        # roll= np.reshape(roll, len(x))
        # pitch = np.reshape(pitch, len(x))
        # yaw = np.reshape(yaw, len(x))


        #MA filter with window size N


        filter=False
        if filter:
            N = 3
            pose_straight=self.poseFilter(pose_straight,N)
            pose_curve=self.poseFilter(pose_curve,N)


        data ={'straight':pose_straight,'curve':pose_curve}

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


        
    def modelfun(self, X, c, cl, tr, x_0, y_0, yaw_0, d):
        Ts=1.0/30 # Sampling Time of Euler Integration step
        
        d = 0.06  # Distance of camera from Baseline is fixed and not part of the optimization for now
        #X=X.reshape((int(X.size/2),2),order='F')
        #cmd_right = X[:,0];
        #cmd_left  = X[:,1];
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
        vx_pred      = c * (cmd_right+cmd_left)*0.5# + tr * (cmd_right-cmd_left)*0.5;
        omega_pred   = cl * (cmd_right-cmd_left)*0.5 + tr * (cmd_right+cmd_left)*0.5;
        vy_pred      = omega_pred*d;  # The model currently also estimates the offset of the camera position
        
        # Forward Euler Integration (improve to RK4?) to get Position Estimates
        yaw_pred = np.cumsum(omega_pred)*Ts + yaw_0;
        x_pred   = np.cumsum(np.cos(yaw_pred)*vx_pred+np.sin(yaw_pred)*vy_pred)*Ts + x_0;
        y_pred   = np.cumsum(np.sin(yaw_pred)*vx_pred+np.cos(yaw_pred)*vy_pred)*Ts + y_0;
        # The _0 Parameters are just the integration constants and also need to be fitted
        # but we don't care about them later
        
        # Match and pick the prediction steps with the closest available position
        # Based on looking at the data we assume the images were taken at a perfect
        # 30 FPS rate. Therefore, we match to the fixed (perfect) sampling rate
        # and do not interpolate. The pred vectors are thus evaluated at the timepoints
        
        # Output has to be a 1D Vector for curve_fit to work
        Y = np.concatenate((x_pred[timepoints], y_pred[timepoints], yaw_pred[timepoints]), axis=0)
        return Y

    def nonlinear_model_fit(self):
        
        x_ramp_meas    = self.veh_pose_['straight']['x_veh']
        x_ramp_meas    = np.reshape(x_ramp_meas,np.size(x_ramp_meas))  # fixing some totally fucked up dimensions
        y_ramp_meas    = self.veh_pose_['straight']['y_veh']
        y_ramp_meas    = np.reshape(y_ramp_meas,np.size(y_ramp_meas))  # fixing some totally fucked up dimensions
        yaw_ramp_meas  = self.veh_pose_['straight']['yaw_veh']
        yaw_ramp_meas  = ( np.array(yaw_ramp_meas) + np.pi*0.5 + np.pi) % (2 * np.pi ) - np.pi # shift by pi/2 and wrap to +-pi
        time_ramp_meas = self.veh_pose_['straight']['timestamp']
        
        for i in range(0,np.size(time_ramp_meas)):  # convert to float
            time_ramp_meas[i] = time_ramp_meas[i].to_sec()
        time_ramp_meas = np.array(time_ramp_meas)

        cmd_ramp_right = np.concatenate([[0],self.wheels_cmd_['vel_r']])  # add a 0 cmd at the beginning
        cmd_ramp_left  = np.concatenate([[0],self.wheels_cmd_['vel_l']])  # as interpolation boundary
        time_ramp_cmd  = self.wheels_cmd_['timestamp']
        
        for i in range(0,np.size(time_ramp_cmd)):  # convert time to float
            time_ramp_cmd[i] = time_ramp_cmd[i].to_sec()
        time_ramp_cmd = np.array(time_ramp_cmd)

        x_sine_meas    = self.veh_pose_['curve']['x_veh']
        x_sine_meas    = np.reshape(x_sine_meas,np.size(x_sine_meas))  # fixing some totally fucked up dimensions
        y_sine_meas    = self.veh_pose_['curve']['y_veh']
        y_sine_meas    = np.reshape(y_sine_meas,np.size(y_sine_meas))  # fixing some totally fucked up dimensions
        yaw_sine_meas  = self.veh_pose_['curve']['yaw_veh']
        yaw_sine_meas  = ( np.array(yaw_sine_meas) + np.pi*0.5 + np.pi) % (2 * np.pi ) - np.pi # shift by pi/2 and wrap to +-pi
        time_sine_meas = self.veh_pose_['curve']['timestamp']
        
        for i in range(0,np.size(time_sine_meas)):  # convert time to float
            time_sine_meas[i] = time_sine_meas[i].to_sec()
        time_sine_meas = np.array(time_sine_meas)

        cmd_sine_right = np.concatenate([[0],self.wheels_cmd_['vel_r']])  # add a 0 cmd at the beginning
        cmd_sine_left  = np.concatenate([[0],self.wheels_cmd_['vel_l']])  # as interpolation boundary
        time_sine_cmd  = self.wheels_cmd_['timestamp']
        
        # NOT NECESSARY AGAIN BECAUSE PYTHON ALLOCATING IS NOT COPYING
        # -> BASICALLY ALL THESE VARIABLES ARE STILL LINKED
        #for i in range(0,np.size(time_sine_cmd)):  # convert to float
        #    time_sine_cmd[i] = time_sine_cmd[i].to_sec()
        #time_sine_cmd = np.array(time_sine_cmd)
        
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

        ### Define if part of the measurement should be cut off
        start = 0
        stop  = -10
        ### IDENTIFICATION FOR RAMP MANOUVER ###
        # Setup Measurements Vectors
        X = np.concatenate((cmd_ramp_right[start:stop], cmd_ramp_left[start:stop], timepoints_ramp[start:stop], [np.size(cmd_ramp_right[start:stop])]), axis=0)
        Y = np.concatenate((x_ramp_meas[start:stop], y_ramp_meas[start:stop], yaw_ramp_meas[start:stop]), axis=0)
        beta0 = np.array([0.6, 6, 0, x_ramp_meas[start], y_ramp_meas[start], yaw_ramp_meas[start], 0.06])
        
        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model predition
        popt_ramp, pcov = curve_fit(self.modelfun, X, Y, p0=beta0)
        print('Optimization Result for Ramp Manouver:',popt_ramp)
        
        # Make a prediction based on the fitted parameters
        y_pred_ramp = self.modelfun(X, *popt_ramp) # Predict to calculate Error
        MSE_ramp = np.sum((Y-y_pred_ramp)**2)/y_pred_ramp.size # Calculate the Mean Squared Error
        X = np.concatenate((cmd_ramp_right[start:stop], cmd_ramp_left[start:stop], np.arange(0,np.size(time_ramp[start:stop]),1), [np.size(cmd_ramp_right[start:stop])]), axis=0)
        y_pred_ramp = self.modelfun(X, *popt_ramp) # Predict for Plotting
        y_pred_ramp = y_pred_ramp.reshape((int(y_pred_ramp.size/3),3),order='F')
        
        
        ### IDENTIFICATION FOR SINE MANOUVER ###
        # Setup Measurements Vectors
        X = np.concatenate((cmd_sine_right[start:stop], cmd_sine_left[start:stop], timepoints_sine[start:stop], [np.size(cmd_sine_right[start:stop])]), axis=0)
        Y = np.concatenate((x_sine_meas[start:stop], y_sine_meas[start:stop], yaw_sine_meas[start:stop]), axis=0)
        beta0 = np.array([0.6, 6, 0, x_sine_meas[start], y_sine_meas[start], yaw_sine_meas[start], 0.06])
        
        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model predition
        popt_sine, pcov = curve_fit(self.modelfun, X, Y, p0=beta0)
        print('Optimization Result for Sine Manouver:',popt_sine)
        
        # Make a prediction based on the fitted parameters
        y_pred_sine = self.modelfun(X, *popt_sine) # Predict to calculate Error
        MSE_sine = np.sum((Y-y_pred_sine)**2)/y_pred_sine.size # Calculate the Mean Squared Error
        X = np.concatenate((cmd_sine_right[start:stop], cmd_sine_left[start:stop], np.arange(0,np.size(time_sine[start:stop]),1), [np.size(cmd_sine_right[start:stop])]), axis=0)
        y_pred_sine = self.modelfun(X, *popt_sine) # Predict for Plotting
        y_pred_sine = y_pred_sine.reshape((int(y_pred_sine.size/3),3),order='F')
        
        
        # PLOTTING
        plt.figure(1)
        plt.plot(time_ramp[timepoints_ramp[start:stop]],x_ramp_meas[start:stop],'x',color=(0.5,0.5,1))
        plt.plot(time_ramp[timepoints_ramp[start:stop]],y_ramp_meas[start:stop],'x',color=(0.5,1,0.5))
        plt.plot(time_ramp[timepoints_ramp[start:stop]],yaw_ramp_meas[start:stop],'x',color=(1,0.5,0.5))
        plt.plot(time_ramp[start:stop],y_pred_ramp[:,0],'b')
        plt.plot(time_ramp[start:stop],y_pred_ramp[:,1],'g')
        plt.plot(time_ramp[start:stop],y_pred_ramp[:,2],'r')
        plt.legend(['x measured','y measured','yaw measured','x predicted','y predicted','yaw predicted'],loc=4)
        plt.title('Measurements vs Prediction - Ramp Manouver')
        plt.xlabel('time [s]')
        plt.ylabel('position [m] / heading [rad]')
        plt.show(block=False)
        
        plt.figure(2)
        plt.plot(time_sine[timepoints_sine[start:stop]],x_sine_meas[start:stop],'x',color=(0.5,0.5,1))
        plt.plot(time_sine[timepoints_sine[start:stop]],y_sine_meas[start:stop],'x',color=(0.5,1,0.5))
        plt.plot(time_sine[timepoints_sine[start:stop]],yaw_sine_meas[start:stop],'x',color=(1,0.5,0.5))
        plt.plot(time_sine[start:stop],y_pred_sine[:,0],'b')
        plt.plot(time_sine[start:stop],y_pred_sine[:,1],'g')
        plt.plot(time_sine[start:stop],y_pred_sine[:,2],'r')
        plt.legend(['x measured','y measured','yaw measured','x predicted','y predicted','yaw predicted'],loc=4)
        plt.title('Measurements vs Prediction - Sine Manouver')
        plt.xlabel('time [s]')
        plt.ylabel('position [m] / heading [rad]')
        plt.show(block=False)
        
        print("\nThe Estimated Model Parameters are:")
        print("c  = {}".format(popt_ramp[0]))  # using ramp estimation
        print("cl = {}".format(popt_sine[1]))  # using sine estimation
        print("tr = {}".format(+popt_ramp[2]))  # using ramp estimation
        print("The Mean Squared Error of the Ramp Manouver is {}".format(MSE_ramp))
        print("The Mean Squared Error of the Sine Manouver is {}".format(MSE_sine))
        
        fit ={'c':popt_ramp[0],'cl':popt_sine[1],'tr':popt_ramp[2]}

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
        baseline = 2 * c / cl
        
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
        #plt.show()


####################
# Main part of the script

if __name__ == '__main__':
    calib=calib()
