#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import message_filters

### note you need to change the name of the robot to yours here
from obst_avoid.detector import Detector
from obst_avoid.visualizer import Visualizer
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics

class ObstDetectNodeVisual(object):
    """
    Visualisation of Obstacle Detection Node
    """
    def __init__(self):
        self.node_name = "Obstacle Detecion Node"
        robot_name = rospy.get_param("~robot_name", "")
        #robot_name = "dori"
        self.show_marker = (rospy.get_param("~show_marker", ""))
        #self.show_marker=True
        self.show_image = (rospy.get_param("~show_image", ""))
        #self.show_image=True

        self.visualizer = Visualizer(robot_name=robot_name)

        # Load camera calibration parameters
	self.intrinsics = load_camera_intrinsics(robot_name)

        # Create Publishers
        if (self.show_marker):
            self.pub_topic_marker = '/{}/obst_detect_visual/visualize_obstacles'.format(robot_name)
            self.publisher_marker = rospy.Publisher(self.pub_topic_marker, MarkerArray, queue_size=1)
            print "YEAH I GIVE YOU THE MARKER"

        if (self.show_image):
                self.pub_topic_img = '/{}/obst_detect_visual/image/compressed'.format(robot_name)
                self.publisher_img = rospy.Publisher(self.pub_topic_img, CompressedImage, queue_size=1)
                print "YEAH I GIVE YOU THE IMAGE"

        # Create necessary Publishers
        if (self.show_image):
                self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
                self.subscriber = message_filters.Subscriber(self.sub_topic, CompressedImage)
        if (self.show_image and not(self.show_marker)):
                self.subscriber.registerCallback(self.callback)
        if (self.show_marker):
                self.sub_topic_arr = '/{}/obst_detect/posearray'.format(robot_name)
                self.subscriber_arr = message_filters.Subscriber(self.sub_topic_arr, PoseArray)
        if (self.show_marker and not(self.show_image))
                self.subscriber_arr.registerCallback(self.callback)
        if (self.show_marker and self.show_image)
                self.ts = message_filters.TimeSynchronizer([self.subscriber_arr,self.subscriber],500)
                self.ts.registerCallback(self.callback)

    def callback(self,obst_list,image):
        print "CALLBACK HERE"
        if (self.show_marker):
                marker_list = self.visualizer.visualize_marker(obst_list)
                self.publisher_marker.publish(marker_list)

        #4. EVENTUALLY DISPLAY IMAGE
        if (self.show_image):
                obst_image = CompressedImage()
                obst_image.header.stamp = image.header.stamp
                obst_image.format = "jpeg"
                obst_image.data = self.visualizer.visualize_image(rectify(rgb_from_ros(image),self.intrinsics),obst_list)
                self.publisher_img.publish(obst_image.data)

  

    def onShutdown(self):
        rospy.loginfo('Shutting down Visualisation of Obstacle Detection')

# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
    rospy.init_node('obst_detection_node_visual', anonymous=False)
    obst_detection_node = ObstDetectNodeVisual()
    rospy.on_shutdown(obst_detection_node.onShutdown)
    rospy.spin()
