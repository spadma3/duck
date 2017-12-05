#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import CompressedImage


class IntersectionLocalization(object):
    def __init__(self):
        # save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing." %(self.node_name))

        # read parameters
        self.robot_name = self.SetupParameter("~robot_name", "daisy")

        # set up subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.ModeCallback, queue_size=1)
        self.sub_img = rospy.Subscriber("/" + self.robot_name + "/camera_node/image/compressed", CompressedImage,
                                        self.ImageCallback, queue_size=1)
        #self.sub_intersection_pose_pred = rospy.Subscriber("~intersection_pose_pred", IntersectionPoseInertial, self.IntersectionPosePredCallback, queue_size=1)

        # set up publishers
        #self.pub_intersection_pose_meas = rospy.Publisher("~intersection_pose_meas", IntersectionPoseInertial, queue_size=1)
        self.sub_img = rospy.Subscriber("/daisy/camera_node/image/compressed", CompressedImage, self.ImageCallback, queue_size=1)

        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def ModeCallback(self,msg):
        # TODO
        # possibly main loop, will need to think about architecture
        pass

    def ImageCallback(self,msg):
        # TODO
        # do localization here
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
    rospy.init_node('intersection_localization_node', anonymous=False)

    # Create the NodeName object
    node = IntersectionLocalization()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.OnShutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
