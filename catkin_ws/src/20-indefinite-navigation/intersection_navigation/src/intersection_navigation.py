#!/usr/bin/env python
import rospy
import cv2
from path_planner.path_planner import PathPlanner
from pose_estimator.pose_estimator import PoseEstimator, VehicleCommand
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import AprilTagsWithInfos, FSMState, TagInfo, Twist2DStamped, BoolStamped, IntersectionPose, IntersectionPoseImg, LanePose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String
import duckietown_utils as dt_utils
import numpy as np
import math


class IntersectionNavigation(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.veh = self.SetupParameter("~veh", "daisy")

        # set up path planner, state estimator, ...
        self.intersectionLocalizer = IntersectionLocalizer(self.veh)
        self.pathPlanner = PathPlanner(self.veh)
        self.poseEstimator = PoseEstimator()
        

        # main logic parameters
        self.rate = 10  # main logic runs at 10Hz
        self.timeout = 1.0

        self.v = 0.15 # Navigation velocity
        self.v_scale = 0.67 * 2.45  # Scaling factor for the velocity
        self.w_scale = 0.45  # Scaling factor for omega

        self.state_dict = dict()
        for counter, key in enumerate(['IDLE', 'INITIALIZING_LOCALIZATION', 'INITIALIZING_PATH', 'TRAVERSING', 'DONE', 'ERROR']):
            self.state_dict.update({key: counter})
        self.state = self.state_dict['IDLE']

        # auxiliary variables
        self.tag_info = TagInfo()
        self.intersection_signs = [self.tag_info.FOUR_WAY, self.tag_info.RIGHT_T_INTERSECT,
                                   self.tag_info.LEFT_T_INTERSECT, self.tag_info.T_INTERSECTION]

        # nominal stop positions: centered in lane, 0.13m in front of center of red stop line, 0 relative orientation error
        self.nominal_start_positions = {self.tag_info.FOUR_WAY: [0.400, -0.105, 0.5 * np.pi],
                                       self.tag_info.LEFT_T_INTERSECT: [0.664, 0.400, np.pi],
                                       self.tag_info.RIGHT_T_INTERSECT: [-0.105, 0.121, 0.0 * np.pi],
                                       self.tag_info.T_INTERSECTION: [0.400, -0.105, 0.5 * np.pi]}
        self.nominal_final_positions = [[0.159, 0.0508, -0.5 * np.pi],
                                        [0.508, 0.159, 0.0],
                                        [0.400, 0.508, 0.5 * np.pi],
                                        [0.0508, 0.400, np.pi]]

        # initializing variables
        self.init_debug = False

        # set up subscribers
        self.sub_mode = rospy.Subscriber("~mode",
                                         FSMState,
                                         self.ModeCallback,
                                         queue_size=1)
        self.sub_turn_type = rospy.Subscriber("~turn_type",
                                              Int16,
                                              self.TurnTypeCallback,
                                              queue_size=1)
        self.sub_img = rospy.Subscriber("~img",
                                        CompressedImage,
                                        self.ImageCallback,
                                        queue_size=1)
        self.sub_pose = rospy.Subscriber("~pose_in",
                                         IntersectionPose,
                                         self.PoseCallback,
                                         queue_size=1)
        self.sub_cmd = rospy.Subscriber("~cmds",
                                        Twist2DStamped,
                                        self.CmdCallback,
                                        queue_size=30)
        '''self.sub_april_tags = rospy.Subscriber('~apriltags',
                                               AprilTagsWithInfos,
                                               self.AprilTagsCallback,
                                               queue_size=1)'''


        # set up publishers
        # self.pub_intersection_pose_pred = rospy.Publisher("~intersection_pose_pred", IntersectionPose queue_size=1)
        self.pub_intersection_pose_img = rospy.Publisher("~pose_img_out", IntersectionPoseImg, queue_size=1)
        self.pub_intersection_pose = rospy.Publisher("~pose", IntersectionPose, queue_size=1)
        self.pub_lane_pose = rospy.Publisher("~intersection_navigation_pose", LanePose, queue_size=1)
        self.pub_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.pub_cmds = rospy.Publisher("~cmds_out", Twist2DStamped, queue_size=1)


        rospy.loginfo("[%s] Initialized." % (self.node_name))


    def ComputeFinalPose(self, intersection_type, turn_type):
        if intersection_type == self.tag_info.FOUR_WAY or intersection_type == self.tag_info.T_INTERSECTION:
            if turn_type == 0: # straight
                return self.nominal_final_positions[2]
            elif turn_type == 1: # left
                return self.nominal_final_positions[3]
            else: # right
                return self.nominal_final_positions[1]

        elif intersection_type == self.tag_info.LEFT_T_INTERSECT:
            if turn_type == 0: # straight
                return self.nominal_final_positions[3]
            else: # left
                return self.nominal_final_positions[0]

        else:
            if turn_type == 0: # straight
                return self.nominal_final_positions[1]
            else: # right
                return self.nominal_final_positions[0]


    def MainLoop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # run state machine
            if self.state == self.state_dict['IDLE']:
                # waiting for FSMState to tell us that Duckiebot is at an intersection (see ModeCallback)
                pass

            elif self.state == self.state_dict['INITIALIZING_LOCALIZATION']:
                if self.InitializeLocalization():
                    self.state = self.state_dict['INITIALIZING_PATH']
                    rospy.loginfo("[%s] Initialized intersection localization, initializing path." % (self.node_name))

            elif self.state == self.state_dict['INITIALIZING_PATH']:
                if self.InitializePath():
                    self.state = self.state_dict['TRAVERSING']
                    self.s = 0.0
                    rospy.loginfo("[%s] Initialized path, traversing intersection." % (self.node_name))
                else:
                    self.state = self.state_dict['ERROR']
                    rospy.loginfo("[%s] Could not initialize path." % (self.node_name))

            elif self.state == self.state_dict['TRAVERSING']:

                if not self.init_debug:
                    self.init_debug = True
                    self.debug_start = rospy.Time.now()

                msg_pose = IntersectionPose()
                msg_pose.header.stamp = rospy.Time.now()
                pose, _ = self.poseEstimator.PredictState(msg_pose.header.stamp)
                msg_pose.x = pose[0]
                msg_pose.y = pose[1]
                msg_pose.theta = pose[2]
                self.pub_intersection_pose.publish(msg_pose)

                '''msg_lanePose = LanePose()
                msg_lanePose.header.stamp = rospy.Time.now()

                if 0.0 < (rospy.Time.now() - self.debug_start).to_sec():

                    dist, theta, curvature, self.s = self.pathPlanner.ComputeLaneError(pose, self.s)

                    if (self.s > 0.999):
                        msg_lanePose.d = 0
                        msg_lanePose.d_ref = 0
                        msg_lanePose.phi = 0
                        msg_lanePose.curvature_ref = 0
                        msg_lanePose.v_ref = 0
                        self.state = self.state_dict['DONE']
                    else:
                        msg_lanePose.d = dist
                        msg_lanePose.d_ref = 0
                        msg_lanePose.phi = theta
                        msg_lanePose.curvature_ref = curvature
                        msg_lanePose.v_ref = 0.15

                else:
                    msg_lanePose.d = 0
                    msg_lanePose.d_ref = 0
                    msg_lanePose.phi = 0
                    msg_lanePose.curvature_ref = 0
                    msg_lanePose.v_ref = 0

                self.pub_lane_pose.publish(msg_lanePose)'''

                msg_cmds = Twist2DStamped()
                msg_cmds.header.stamp = rospy.Time.now()

                if 2.0 < (rospy.Time.now() - self.debug_start).to_sec() and (rospy.Time.now() - self.debug_start).to_sec() < 10.4: #Wait a bit before starting

                    '''pos, vel = self.pathPlanner.EvaluatePath(self.s)
                    dt = 0.01
                    _, vel2 = self.pathPlanner.EvaluatePath(self.s + dt)
                    self.alpha = 0.15/np.linalg.norm(vel)

                    dir = vel/np.linalg.norm(vel)
                    dir2 = vel2/np.linalg.norm(vel2)
                    theta = np.arctan2(dir[1],dir[0])
                    theta2 = np.arctan2(dir2[1], dir2[0])
                    omega = (theta2 - theta)/dt

                    msg_cmds.v = 0.15
                    msg_cmds.omega = self.alpha * omega'''

                    msg_cmds.v = 0.15
                    msg_cmds.omega = 0.15/0.2
                    if (msg_cmds.v - 0.5 * math.fabs(msg_cmds.omega) * 0.1) < 0.061:
                        msg_cmds.v = 0.061 + 0.5 * math.fabs(msg_cmds.omega) * 0.1
                        #self.alpha = self.alpha*msg_cmds.v/0.15

                    msg_cmds.v = msg_cmds.v * 1.467
                    msg_cmds.omega = msg_cmds.omega * 4.24

                    #self.s = self.s + self.alpha*(rospy.Time.now() - self.debug_time).to_sec()

                    '''if (self.s > 1.0):
                        msg_cmds.v = 0.0
                        msg_cmds.omega = 0.0
                        self.state = self.state_dict['DONE']'''

                else:
                    msg_cmds.v = 0.0
                    msg_cmds.omega = 0.0

                self.debug_time = rospy.Time.now()
                self.pub_cmds.publish(msg_cmds)

                '''elif self.state == self.state_dict['DONE']:
                    # Now just stop
                    msg_done_cmds = Twist2DStamped()
                    msg_done_cmds.header.stamp = rospy.Time.now()
                    msg_done_cmds.v = 0.0
                    msg_done_cmds.omega = 0.0
    
                    msg_done_cmds = LanePose()
                    msg_done_cmds.header.stamp = rospy.Time.now()
                    msg_done_cmds.d = 0
                    msg_done_cmds.d_ref = 0
                    msg_done_cmds.phi = 0
                    msg_done_cmds.curvature_ref = 0
                    msg_done_cmds.v_ref = 0
    
                    self.pub_lane_pose.publish(msg_done_cmds)
                    #self.pub_cmds.publish(msg_done_cmds)
                    rospy.loginfo("[%s] Intersection done." % (self.node_name))'''

            else:
                pass
                # TODO

            rate.sleep()

    def InitializeLocalization(self):
        # waiting for april tag info (type intersection and which exit)
        try:
            april_msg = rospy.wait_for_message('~apriltags_out', AprilTagsWithInfos, self.timeout)
        except rospy.ROSException:
            rospy.loginfo("[%s] Timeout waiting for april tag info." % (self.node_name))
            return False

        # find closest (valid) april tag
        closest_distance = 1e6
        closest_idx = -1
        for idx, detection in enumerate(april_msg.detections):
            if april_msg.infos[idx].tag_type == self.tag_info.SIGN:
                if april_msg.infos[idx].traffic_sign_type in self.intersection_signs:
                    position = detection.pose.pose.position
                    distance = np.sqrt(position.x ** 2 + position.y ** 2 + position.z ** 2)

                    if distance < closest_distance:
                        closest_distance = distance
                        closest_idx = idx

        # return if no valid april tag was found
        if closest_idx < 0:
            return False

        # initial position estimate
        x_init = self.nominal_start_positions[april_msg.infos[closest_idx].traffic_sign_type][0]
        y_init = self.nominal_start_positions[april_msg.infos[closest_idx].traffic_sign_type][1]
        theta_init = self.nominal_start_positions[april_msg.infos[closest_idx].traffic_sign_type][2]

        if april_msg.infos[closest_idx].traffic_sign_type in [self.tag_info.RIGHT_T_INTERSECT,
                                                              self.tag_info.LEFT_T_INTERSECT]:
            dx_init = np.linspace(-0.03, 0.03, 7)
            dy_init = np.linspace(-0.05, 0.05, 11)
            dtheta_init = np.linspace(-20.0 / 180.0 * np.pi, 20.0 / 180.0 * np.pi, 5)
        else:
            dx_init = np.linspace(-0.05, 0.05, 11)
            dy_init = np.linspace(-0.03, 0.03, 7)
            dtheta_init = np.linspace(-20.0 / 180.0 * np.pi, 20.0 / 180.0 * np.pi, 5)

        if april_msg.infos[closest_idx].traffic_sign_type == self.tag_info.FOUR_WAY:
            self.intersectionLocalizer.SetEdgeModel('FOUR_WAY_INTERSECTION')
        else:
            self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')

        '''x_init = self.nominal_start_positions[self.tag_info.T_INTERSECTION][0]
        y_init = self.nominal_start_positions[self.tag_info.T_INTERSECTION][1]
        theta_init = self.nominal_start_positions[self.tag_info.T_INTERSECTION][2]

        dx_init = np.linspace(-0.05, 0.05, 11)
        dy_init = np.linspace(-0.03, 0.03, 7)
        dtheta_init = np.linspace(-10.0 / 180.0 * np.pi, 10.0 / 180.0 * np.pi, 3)

        self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')'''

        #rospy.set_param("/daisy/lane_controller_node/v_ref", 0.3)

        # waiting for camera image
        try:
            img_msg = rospy.wait_for_message("~img",
                                             CompressedImage, self.timeout)
        except rospy.ROSException:
            rospy.loginfo("[%s] Timeout waiting for camera image." % (self.node_name))
            return False

        # initialize intersection localizer
        img_processed, img_gray = self.intersectionLocalizer.ProcessRawImage(img_msg)

        best_likelihood = -1.0
        best_pose_meas = np.zeros(3,float)
        pose = np.zeros(3,float)
        for dx in dx_init:
            for dy in dy_init:
                for dtheta in dtheta_init:
                    pose[0] = x_init + dx
                    pose[1] = y_init + dy
                    pose[2] = theta_init + dtheta
                    valid_meas, pose_meas, likelihood = self.intersectionLocalizer.ComputePose(img_processed, pose)

                    if valid_meas and likelihood > best_likelihood:
                        best_likelihood = likelihood
                        best_pose_meas[:] = pose_meas

        if best_likelihood < 0.0:
            rospy.loginfo("[%s] Could not initialize intersection localizer." % (self.node_name))
            return False

        msg = IntersectionPose()
        msg.header.stamp = rospy.Time.now()
        msg.x = best_pose_meas[0]
        msg.y = best_pose_meas[1]
        msg.theta = best_pose_meas[2]
        self.pub_intersection_pose.publish(msg)

        self.poseEstimator.Reset(best_pose_meas, img_msg.header.stamp)
        return True

    def InitializePath(self):
        # waiting for instructions where to go
        # TODO
        turn_type = 2

        # 0: straight, 1: left, 2: right
        pose_init, _ = self.poseEstimator.PredictState(rospy.Time.now())
        pose_final = self.ComputeFinalPose(self.tag_info.T_INTERSECTION, turn_type)

        if not self.pathPlanner.PlanPath(pose_init, pose_final):
            rospy.loginfo("[%s] Could not compute feasible path." % (self.node_name))
            return False

        else:
            return True


    def ModeCallback(self, msg):
        # update state if we are at an intersection
        if self.state == self.state_dict['IDLE'] and msg.state == "INTERSECTION_CONTROL":
            #self.state = self.state_dict['INITIALIZING_LOCALIZATION']
            self.state = self.state_dict['TRAVERSING']
            rospy.loginfo("[%s] Arrived at intersection, initializing intersection localization." % (self.node_name))
            
    def TurnTypeCallback(self, msg):
        # TODO
        pass


    def ImageCallback(self, msg):
        if self.state == self.state_dict['TRAVERSING']:
            # predict pose
            pose_pred, _ = self.poseEstimator.PredictState(msg.header.stamp)
            msg_out = IntersectionPoseImg()
            msg_out.header.stamp = msg.header.stamp
            msg_out.x = pose_pred[0]
            msg_out.y = pose_pred[1]
            msg_out.theta = pose_pred[2]
            msg_out.img = msg
            self.pub_intersection_pose_img.publish(msg_out)

    def PoseCallback(self, msg):
        '''pose_meas = np.array([msg.x, msg.y, msg.theta])
        self.poseEstimator.UpdateWithPoseMeasurement(pose_meas, 1e-7*np.diag([1.0,1.0,1.0]), msg.header.stamp)
        pass'''


    def CmdCallback(self, msg):
        if self.state == self.state_dict['INITIALIZING_PATH'] or self.state == self.state_dict['TRAVERSING']:
            cmd_msg = Twist2DStamped()
            cmd_msg.v = msg.v / 1.467
            cmd_msg.omega = msg.omega /4.24
            cmd_msg.header.stamp = msg.header.stamp
            self.poseEstimator.FeedCommandQueue(cmd_msg)


    def AprilTagsCallback(self, msg):
        '''if self.state == self.state_dict['IDLE'] or self.state == self.state_dict['INITIALIZING']:
            pass'''
        pass

    def SetupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # initialize the node with rospy
    rospy.init_node('intersection_navigation_node', anonymous=False)

    # create the intersection navigation object
    node = IntersectionNavigation()

    # setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # run main logic
    node.MainLoop()
