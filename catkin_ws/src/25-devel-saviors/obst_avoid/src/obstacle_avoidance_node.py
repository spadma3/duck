#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray
from duckietown_msgs.msg import LanePose

### note you need to change the name of the robot to yours here
# from obst_avoid.detector import Detector
from obst_avoid.avoider import Avoider
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics

class ObstAvoidNode(object):

    def __init__(self):
        self.node_name = "Obstacle Avoidance Node"
        self.d_current = 0
        self.theta_current = 0
        self.intersection = 0
        self.x_bounding_width = (rospy.get_param("~bb_len", ""))  # mm
        self.y_bounding_width = (rospy.get_param("~bb_wid", ""))  # mm

        robot_name = rospy.get_param("~veh", "")
        self.avoider = Avoider(robot_name=robot_name)
        rospy.loginfo(robot_name)
        # self.detector = Detector(robot_name=robot_name)  # not sure what that does

        ########################
        ###### Publishers ######
        # Emergency brake to be triggered iff == 1
        self.pub_topic = 'obstacle_emergency_stop_flag'.format(robot_name)
        self.brake_pub = rospy.Publisher(self.pub_topic, Bool, queue_size=1)

        self.pub_topic = 'obstacle_avoidance_active_flag'.format(robot_name)
        self.brake_pub = rospy.Publisher(self.pub_topic, Bool, queue_size=1)

        # Target d. Only read when Obstacle is detected
        self.pub_topic = '/{}/obst_avoid/d_target'.format(robot_name)
        self.d_target_pub = rospy.Publisher(self.pub_topic, Float32, queue_size=1)
        # ToDo: Consider Float32MultiArray if theta will be used.
        # Target theta for dev-controllers to tune the controls
        self.pub_topic = '/{}/obst_avoid/theta_target'.format(robot_name)
        self.theta_target_pub = rospy.Publisher(self.pub_topic, Float32, queue_size=1)

        ########################
        ###### Subscribers #####
        self.sub_topic = '/{}/obst_detect/posearray'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, PoseArray, self.obstacleCallback)

        # ToDo: d_current, theta_current
        self.sub_topic = '/{}/lane_filter_node/lane_pose/'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, LanePose, self.LanePoseCallback)

        # ToDo: intersection
        # self.sub_topic = '/{}/'.format(robot_name)
        # self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.intersectionCallback)

        # ToDo: line_detection
        self.sub_topic = '/{}/Anti_Instagram'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.lineCallback)

    def obstacleCallback(self, obstacle_poses):
        amount_obstacles = len(obstacle_poses.poses)
        amount_obstacles_on_track = 0
        rospy.loginfo('Number of obstacles: %d', len(obstacle_poses.poses))
        amount_obstacles_on_track = 0
        obstacle_poses_on_track = PoseArray()
        obstacle_poses_on_track.header = obstacle_poses.header
        for x in range(amount_obstacles):
            rospy.loginfo(obstacle_poses.poses[x].position.z)
            if obstacle_poses.poses[x].position.z > 0:
                # Bounding window
                # get relative coordinates
                x_obstacle = obstacle_poses.poses[x].position.x * 1000  # mm
                y_obstacle = obstacle_poses.poses[x].position.y * 1000  # mm
                # get global coordinates
                global_pos_vec = self.avoider.coordinatetransform(x_obstacle, y_obstacle,
                                                                  -self.theta_current, self.d_current)
                x_global = global_pos_vec[0]  # mm
                y_global = global_pos_vec[1]  # mm
                # check if obstacle is within boundaries
                if x_global < self.x_bounding_width and abs(y_global) < self.y_bounding_width:
                    rospy.loginfo('Obstacle in range - Beware')
                    obstacle_poses_on_track.poses.append(obstacle_poses.poses[x])
                    amount_obstacles_on_track += 1
        if amount_obstacles_on_track == 0:
            self.brake_pub.publish(0)
            rospy.loginfo('0 obstacles on track')
            return
        if amount_obstacles_on_track == 1:
            #ToDo: check if self.d_current can be accessed through forwarding of self
            targets = self.avoider.avoid(obstacle_poses_on_track, self.d_current, self.theta_current)
            self.d_target_pub.publish(targets[0])
            self.brake_pub.publish(targets[1])
            # self.theta_target_pub.publish(targets[2]) # theta not calculated in current version
            rospy.loginfo('d_target= %f', targets[0])
            rospy.loginfo('emergency_stop = %f', targets[1])
            rospy.loginfo('1 obstacles on track')
        else:
            self.brake_pub.publish(1)
            rospy.loginfo('%d obstacles on track', amount_obstacles_on_track)
        return

    def LanePoseCallback(self, LanePose):
        self.d_current = LanePose.d
        rospy.loginfo('Current d: %f', self.d_current)
        self.theta_current = LanePose.phi
        rospy.loginfo('Current theta: %f', self.theta_current)
        return

    def intersectionCallback(self, intersection_update):
        self.intersection = intersection_update
        return

    def lineCallback(self):
        return

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')


if __name__ == '__main__':
    rospy.init_node('obst_avoidance_node', anonymous=False)
    obst_avoidance_node = ObstAvoidNode()
    rospy.on_shutdown(obst_avoidance_node.onShutdown)
    rospy.spin()