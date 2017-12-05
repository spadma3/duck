#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import CompressedImage


class IntersectionNavigation(object):
    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." %(self.node_name))

        # read parameters
        self.robot_name = self.SetupParameter("~robot_name", "daisy")

        # set up path planner, state estimator, ...
        # self.pathPlanner = ...
        # self.stateEstimator = ...

        # set up subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.ModeCallback, queue_size=1)
        self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.TurnTypeCallback, queue_size=1)
        self.sub_img = rospy.Subscriber("/" + self.robot_name + "/camera_node/image/compressed", CompressedImage, self.ImageCallback, queue_size=1)
        #self.sub_intersection_pose_meas = rospy.Subscriber("~intersection_pose_meas", IntersectionPoseInertial, self.stateEstimator.Update, queue_size=1)
        #self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.stateEstimator.FeedCommands, queue_size = 10)

        # set up publishers
        #self.pub_intersection_pose_pred = rospy.Publisher("~intersection_pose_pred", IntersectionPoseInertial, queue_size=1)
        self.pub_intersection_pose = rospy.Publisher("~intersection_pose", LanePose, queue_size=1)
        self.pub_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)


        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def ModeCallback(self,msg):
        # TODO
        # main loop will be implemented here
        pass

    def TurnTypeCallback(self,msg):
        # TODO
        # will be used to proceed with main loop
        pass

    def ImageCallback(self,msg):
        # TODO
        # predict state estimate until image timestamp and publish "~intersection_pose_pred"
        pass

    def SetupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def OnShutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('intersection_navigation_node', anonymous=False)

    # Create the NodeName object
    node = IntersectionNavigation()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()