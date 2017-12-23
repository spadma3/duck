#!/usr/bin/env python
import rospy
import cv2
from pose_estimator.pose_estimator import PoseEstimator
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import AprilTagsWithInfos, FSMState, TagInfo, Twist2DStamped
from geometry_msgs.msg import PoseStamped
import numpy as np


class IntersectionNavigation(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.robot_name = self.SetupParameter("~robot_name", "daisy")

        # set up path planner, state estimator, ...
        self.intersectionLocalizer = IntersectionLocalizer(self.robot_name)
        # self.pathPlanner = ...
        self.poseEstimator = PoseEstimator()

        # set up subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.ModeCallback, queue_size=1)
        # self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.TurnTypeCallback, queue_size=1)
        self.sub_img = rospy.Subscriber("/" + self.robot_name + "/camera_node/image/compressed", CompressedImage,
                                        self.ImageCallback, queue_size=1)
        # self.sub_intersection_pose_meas = rospy.Subscriber("~intersection_pose_meas", IntersectionPoseInertial, self.stateEstimator.Update, queue_size=1)
        self.sub_car_cmd = rospy.Subscriber("/" + self.robot_name + "/joy_mapper_node/car_cmd", Twist2DStamped,
                                            self.poseEstimator.FeedCommandQueue, queue_size=10)

        # set up publishers
        # self.pub_intersection_pose_pred = rospy.Publisher("~intersection_pose_pred", IntersectionPoseInertial, queue_size=1)
        # self.pub_intersection_pose = rospy.Publisher("~intersection_pose", LanePose, queue_size=1)
        # self.pub_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)

        # main logic parameters
        self.rate = 10  # main logic runs at 10Hz
        self.timeout = 1.0
        self.state_dict = dict()
        for counter, key in enumerate(['WAITING', 'INITIALIZING', 'WAITING_FOR_INSTRUCTIONS', 'TRAVERSING', 'DONE']):
            self.state_dict.update({key: counter})
        self.state = self.state_dict['WAITING']

        # auxiliary variables
        self.tag_info = TagInfo()
        self.intersection_signs = [self.tag_info.FOUR_WAY, self.tag_info.RIGHT_T_INTERSECT,
                                   self.tag_info.LEFT_T_INTERSECT, self.tag_info.T_INTERSECTION]

        # nominal stop positions: centered in lane, 0.13m in front of center of red stop line, 0 relative orientation error
        self.nominal_stop_positions = {self.tag_info.FOUR_WAY: [0.400, -0.105, 0.5 * np.pi],
                                       self.tag_info.LEFT_T_INTERSECT: [0.664, 0.400, np.pi],
                                       self.tag_info.RIGHT_T_INTERSECT: [-0.105, 0.121, 0.0 * np.pi],
                                       self.tag_info.T_INTERSECTION: [0.400, -0.105, 0.5 * np.pi]}

        rospy.loginfo("[%s] Initialized." % (self.node_name))

    def MainLoop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # run state machine
            if self.state == self.state_dict['WAITING']:
                # waiting for FSMState to tell us that Duckiebot is at an intersection (see ModeCallback)
                continue

            elif self.state == self.state_dict['INITIALIZING']:
                # waiting for april tag info (type intersection and which exit)
                try:
                    msg = rospy.wait_for_message('~apriltags_out', AprilTagsWithInfos, self.timeout)
                except rospy.ROSException:
                    rospy.loginfo("[%s] Timeout waiting for april tag info." % (self.node_name))
                    continue

                # find closest (valid) april tag
                closest_distance = 1e6
                closest_idx = -1
                for idx, detection in enumerate(msg.detections):
                    if msg.infos[idx].tag_type == self.tag_info.SIGN:
                        if msg.infos[idx].traffic_sign_type in self.intersection_signs:
                            position = detection.pose.pose.position
                            distance = np.sqrt(position.x ** 2 + position.y ** 2 + position.z ** 2)

                            if distance < closest_distance:
                                closest_distance = distance
                                closest_idx = idx

                # return if no valid april tag was found
                if closest_idx < 0:
                    continue

                # initial position estimate
                x_init = self.nominal_stop_positions[msg.infos[closest_idx].traffic_sign_type][0]
                y_init = self.nominal_stop_positions[msg.infos[closest_idx].traffic_sign_type][1]
                theta_init = self.nominal_stop_positions[msg.infos[closest_idx].traffic_sign_type][2]

                if msg.infos[closest_idx].traffic_sign_type in [self.tag_info.RIGHT_T_INTERSECT,
                                                                self.tag_info.LEFT_T_INTERSECT]:
                    dx_init = np.linspace(-0.03, 0.03, 7)
                    dy_init = np.linspace(-0.05, 0.05, 11)
                    dtheta_init = np.linspace(-20.0 / 180.0 * np.pi, 20.0 / 180.0 * np.pi, 5)
                else:
                    dx_init = np.linspace(-0.05, 0.05, 11)
                    dy_init = np.linspace(-0.03, 0.03, 7)
                    dtheta_init = np.linspace(-20.0 / 180.0 * np.pi, 20.0 / 180.0 * np.pi, 5)

                if msg.infos[closest_idx].traffic_sign_type == self.tag_info.FOUR_WAY:
                    self.intersectionLocalizer.SetEdgeModel('FOUR_WAY_INTERSECTION')
                else:
                    self.intersectionLocalizer.SetEdgeModel('THREE_WAY_INTERSECTION')

                # debugging
                if 1:
                    if msg.infos[closest_idx].traffic_sign_type == self.tag_info.FOUR_WAY:
                        print('four way intersection')
                    elif msg.infos[closest_idx].traffic_sign_type == self.tag_info.RIGHT_T_INTERSECT:
                        print('right t intersection')
                    elif msg.infos[closest_idx].traffic_sign_type == self.tag_info.LEFT_T_INTERSECT:
                        print('left t intersection')
                    else:
                        print('t intersection')

                    print(x_init)
                    print(y_init)
                    print(theta_init)
                    print('----------------')

                # waiting for camera image
                try:
                    msg = rospy.wait_for_message("/" + self.robot_name + "/camera_node/image/compressed",
                                                 CompressedImage, self.timeout)
                except rospy.ROSException:
                    rospy.loginfo("[%s] Timeout waiting for camera image." % (self.node_name))
                    continue

                # initialize intersection localizer
                img_processed, img_gray = self.intersectionLocalizer.ProcessRawImage(msg)

                best_likelihood = -1.0
                for dx in dx_init:
                    for dy in dy_init:
                        for dtheta in dtheta_init:
                            valid_meas, x_meas, y_meas, theta_meas, likelihood = self.intersectionLocalizer.ComputePose(
                                img_processed,
                                x_init + dx,
                                y_init + dy,
                                theta_init + dtheta)

                            if valid_meas and likelihood > best_likelihood:
                                best_likelihood = likelihood
                                best_x_meas = x_meas
                                best_y_meas = y_meas
                                best_theta_meas = theta_meas

                if best_likelihood < 0.0:
                    rospy.loginfo("[%s] Could not initialize intersection localizer." % (self.node_name))
                    continue

                # waiting for instructions where to go
                # TODO



                # debugging
                if 0:
                    self.intersectionLocalizer.DrawModel(img_gray, best_x_meas, best_y_meas, best_theta_meas)
                    cv2.imshow('initialization', img_gray)
                    cv2.waitKey(1000)


            elif self.state == self.state_dict['TRAVERSING']:
                pass

            elif self.state == self.state_dict['DONE']:
                pass

            else:
                pass
                # TODO

            rate.sleep()

    def ModeCallback(self, msg):
        # update state if we are at an intersection
        if self.state == self.state_dict['WAITING'] and msg.state == "INTERSECTION_CONTROL":
            self.state = self.state_dict['INITIALIZING']

    def TurnTypeCallback(self, msg):
        # TODO
        # will be used to proceed with main loop
        pass

    def ImageCallback(self, msg):
        # TODO
        # predict state estimate until image timestamp and publish "~intersection_pose_pred"
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
