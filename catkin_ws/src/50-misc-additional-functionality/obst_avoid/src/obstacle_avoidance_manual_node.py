#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray
from duckietown_msgs.msg import LanePose, BoolStamped
from obst_avoid.avoider import Avoider
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics


class ObstAvoidNode(object):

    def __init__(self):
        self.node_name = "Obstacle Avoidance Node"
        self.active = False

        self.current_lane_pose = LanePose()
        robot_name = rospy.get_param("~veh", "")
        rospy.loginfo(robot_name)
        # self.detector = Detector(robot_name=robot_name)  # not sure what that does

        ########################
        ###### Publishers ######
        # Emergency brake to be triggered iff == 1
        #self.pub_topic = 'obstacle_emergency_stop_flag'.format(robot_name)
        #self.brake_pub = rospy.Publisher(self.pub_topic, Bool, queue_size=1)
        self.pub_topic = "~obstacle_avoidance_active_flag"
        self.avoid_pub = rospy.Publisher(self.pub_topic, BoolStamped, queue_size=1)

        # Target d. Only read when Obstacle is detected
        self.pub_topic = '~obstacle_avoidance_pose'
        self.obstavoidpose_topic = rospy.Publisher(self.pub_topic, LanePose, queue_size=1)

        ########################
        ###### Subscribers #####
        self.sub_topic = "~start_avoidance"
        self.subscriber = rospy.Subscriber(self.sub_topic, BoolStamped, self.takeoverCallback)

        # ToDo: d_current, theta_current
        self.sub_topic = '~lane_pose'
        self.subscriber = rospy.Subscriber(self.sub_topic, LanePose, self.LanePoseCallback)

        # ToDo: intersection
        # self.sub_topic = '/{}/'.format(robot_name)
        # self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.intersectionCallback)

    def takeoverCallback(self, avoidance_msg):
        self.active = False
        if avoidance_msg.data == True:
            self.active = True
        return

    def LanePoseCallback(self, LanePose):
        self.current_lane_pose = LanePose
        #print(LanePose)
        avoidance_active = BoolStamped()
        avoidance_active.data = False
        LanePose.d_ref = 0
        LanePose.v_ref = 5
        if self.active == True:
            LanePose.d_ref = 0.15
            LanePose.v_ref = 10
            rospy.loginfo('Reference d set: %s', LanePose.d_ref )
            avoidance_active.data = True
            self.active = False
        self.obstavoidpose_topic.publish(LanePose)
        self.avoid_pub.publish(avoidance_active)
        return

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')


if __name__ == '__main__':
    rospy.init_node('obst_avoidance_node', anonymous=False)
    obst_avoidance_node = ObstAvoidNode()
    rospy.on_shutdown(obst_avoidance_node.onShutdown)
    rospy.spin()
