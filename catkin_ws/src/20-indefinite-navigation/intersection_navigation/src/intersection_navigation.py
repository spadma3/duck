#!/usr/bin/env python
import rospy
import cv2
from path_planner.path_planner import PathPlanner
from pose_estimator.pose_estimator import PoseEstimator, VehicleCommands
from intersection_localizer.intersection_localizer import IntersectionLocalizer
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LanePose, AprilTagsWithInfos, FSMState, TagInfo, Twist2DStamped, BoolStamped, IntersectionPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16, String
import numpy as np


class IntersectionNavigation(object):
    '''class that handles the navigation of the Duckiebot at an intersection'''

    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # read parameters
        self.robot_name = self.SetupParameter("~robot_name", "bob")

        # set up path planner, state estimator, ...
        self.intersectionLocalizer = IntersectionLocalizer(self.robot_name)
        self.pathPlanner = PathPlanner(self.robot_name)
        self.poseEstimator = PoseEstimator()
        

        # set up subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.ModeCallback, queue_size=1)
        self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.TurnTypeCallback, queue_size=1)
        self.sub_img = rospy.Subscriber("/" + self.robot_name + "/camera_node/image/compressed", CompressedImage,
                                        self.ImageCallback, queue_size=1)
        self.sub_intersection_pose_meas = rospy.Subscriber("~intersection_pose_meas", IntersectionPose, 
                                        self.poseEstimator.UpdateWithPoseMeasurement, queue_size=1)
        self.sub_car_cmd = rospy.Subscriber("/" + self.robot_name + "/joy_mapper_node/car_cmd", Twist2DStamped, self.CarCmdCallback, queue_size=1)
         
        # self.poseEstimator.FeedCommandQueue, queue_size=10)
        # self.on_regular_road = rospy.Subscriber("~in_lane", BoolStamped, self.ModeCallback, queue_size=1)

        # set up publishers
        # self.pub_intersection_pose_pred = rospy.Publisher("~intersection_pose_pred", IntersectionPose queue_size=1)
        self.pub_intersection_pose = rospy.Publisher("~intersection_pose", LanePose, queue_size=1)
        self.pub_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)

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
        #self.VehicleCommands = VehicleCommands()

        # nominal stop positions: centered in lane, 0.13m in front of center of red stop line, 0 relative orientation error
        self.nominal_stop_positions = {self.tag_info.FOUR_WAY: [0.400, -0.105, 0.5 * np.pi],
                                       self.tag_info.LEFT_T_INTERSECT: [0.664, 0.400, np.pi],
                                       self.tag_info.RIGHT_T_INTERSECT: [-0.105, 0.121, 0.0 * np.pi],
                                       self.tag_info.T_INTERSECTION: [0.400, -0.105, 0.5 * np.pi]}
        self.nominal_final_positions = [[0.159, 0.0508, -0.5 * np.pi],
                                        [0.508, 0.159, 0.0],
                                        [0.400, 0.508, 0.5 * np.pi],
                                        [0.0508, 0.400, np.pi]]

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
                
    def CarCmdCallback(self,cmd_msg):
        return cmd_msg

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
                    april_msg = rospy.wait_for_message('~apriltags_out', AprilTagsWithInfos, self.timeout)
                except rospy.ROSException:
                    rospy.loginfo("[%s] Timeout waiting for april tag info." % (self.node_name))
                    continue

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
                    continue

                # initial position estimate
                x_init = self.nominal_stop_positions[april_msg.infos[closest_idx].traffic_sign_type][0]
                y_init = self.nominal_stop_positions[april_msg.infos[closest_idx].traffic_sign_type][1]
                theta_init = self.nominal_stop_positions[april_msg.infos[closest_idx].traffic_sign_type][2]

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

                # waiting for camera image
                try:
                    img_msg = rospy.wait_for_message("/" + self.robot_name + "/camera_node/image/compressed",
                                                 CompressedImage, self.timeout)
                except rospy.ROSException:
                    rospy.loginfo("[%s] Timeout waiting for camera image." % (self.node_name))
                    continue

                # initialize intersection localizer
                img_processed, img_gray = self.intersectionLocalizer.ProcessRawImage(img_msg)

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
                self.turn_type = TurnTypeCallback()
                # 0: straight, 1: left, 2: right
                self.turn_type = 2            
                pose_init = [best_x_meas, best_y_meas, best_theta_meas]
                pose_final = self.ComputeFinalPose(april_msg.infos[closest_idx].traffic_sign_type, turn_type)

                alphas_init = np.linspace(0.1, 1.6, 11)
                alphas_final = np.linspace(0.1, 1.6, 11)

                # Path Planner
                self.pathPlanner.PlanPath(pose_init, alphas_init, pose_final, alphas_final)
                numPathPoints = 10
                s = np.linnspace(0.0, 1.0, numPathPoints)
                pathPoints, _ = self.pathPlanner.path.Evaluate(s)
                trackingPoints = 0
                currPos = pose_init
                self.state == self.state_dict['TRAVERSING']
                # print(pose_init)
                # print(pose_final)

                # debugging
                '''
                if 1:
                    self.intersectionLocalizer.DrawModel(img_gray, best_x_meas, best_y_meas, best_theta_meas)
                    self.pathPlanner.DrawPath(img_gray, [best_x_meas, best_y_meas, best_theta_meas])
                    cv2.imshow('initialization', img_gray)
                    cv2.waitKey(10)
				'''

            elif self.state == self.state_dict['TRAVERSING']:
                    trackingPoints += 1

                    # Path updating
                    dPath_x = pathPoints[0,trackingPoints] - pathPoints[0,trackingPoints-1]
                    dPath_y = pathPoints[1,trackingPoints] - pathPoints[1,trackingPoints-1]
                    dPath_sqrt = dPath_x*dPath_x + dPath_y*dPath_y
                    dPathRobot_x = pathPoints[0,trackingPoints] - currPos[0]
                    dPathRobot_y = pathPoints[1,trackingPoints] - currPos[1]
                    dPathRobot_sqrt = dPathRobot_x*dPathRobot_x + dPathRobot_y*dPathRobot_y

                    # LanePose message
                    pathTracker_msg = LanePose()
                    pathTracker_msg.d = np.sqrt(dPathRobot_sqrt-dPath_sqrt)
                    pathTracker_msg.phi = np.arctan2(dPath_y,dPath_x) - currPos[3]
					
					#TODO add path curvature. Now we use the standard parameters of the lane following node
                    if self.turn_type == 1:
                        curvature = 0.025
                    elif self.turn_type == 2:
                        curvature = -0.054
                    else:
                        curvature = 0

                    pathTracker_msg.curvature = curvature

                    pathTracker.header.stamp = rospy.Time.now()
                    self.pub_intersection_pose.publish(pathTracker_msg)

                    #Update pose estimation (Only 1 cmd in queue)
                    self.poseEstimator.state_est = currPos
                    self.poseEstimator.cmd_queue = deque([VehicleCommands(cmd_msg.v, cmd_msg.omega, cmd_msg.header.stamp)])

                    dt = rospy.Time.now().secs
                    time_pred = dt - cmd_msg.header.stamp
                    currPos, _ = self.poseEstimator.PredictState(time_pred)

                    if trackingPoints == numPathPoints - 1:
                        self.on_regular_road == True

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

        # update state if we are done with the navigation
        # if self.on_regular_road == True and msg.state == "TRAVERSING":
        if self.on_regular_road == True and msg.state == "INTERSECTION_CONTROL" and self.state == self.state_dict['TRAVERSING']:
            self.state = self.state_dict['DONE']
            in_lane_msg = BoolStamped() # is a header needed in this case ??
            in_lane_msg.header.stamp = rospy.Time.now()
            in_lane_msg.data = True
            self.pub_done.publish(in_lane_msg)
            rospy.loginfo("[%s] Intersection naviation done.")
            

    def TurnTypeCallback(self, msg):
        # TODO
        # will be used to proceed with main loop
        if self.sub_turn_type == 1: # straight
            self.turn_type = 0
        elif self.sub_turn_type == 2: # right
            self.turn_type = 2
        elif self.sub_turn_type == 0: # left
            self.turn_type = 1
        else:
            pass

    def ImageCallback(self, msg):
        # TODO
        # gathering the predicted measurements form PoseEstimator
        state_est, cov_est = PredictState()
        x_pred = state_est[0]
        y_pred = state_est[1]
        theta_pred = state_est[2]

        msg_pose_pred = IntersectionPose()
        # msg_pose_pred.header.stamp = msg_img.header.stamp
        msg_pose_pred.x = x_pred
        msg_pose_pred.y = y_pred
        msg_pose_meas.theta = theta_pred

        #still needs a while loop until image timestamps or something like that

        # predict state estimate until image timestamp and publish "~intersection_pose_pred"
        self.pub_intersection_pose_pred.publish(msg_pose_pred)

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
