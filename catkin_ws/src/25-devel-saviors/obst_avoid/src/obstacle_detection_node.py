#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray

### note you need to change the name of the robot to yours here
from obst_avoid.detector import Detector
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics

class ObstDetectNode(object):
    """
    Obstacle Detection Node
    """
    def __init__(self):
        self.node_name = "Obstacle Detecion Node"
        robot_name = rospy.get_param("~robot_name", "")
        self.count = 1

        self.detector = Detector(robot_name=robot_name)

        # Load camera calibration parameters
	self.intrinsics = load_camera_intrinsics(robot_name)

        # Create a Publisher
        self.pub_topic = '/{}/obst_detect/image/compressed'.format(robot_name)
        self.publisher = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)

        # Create a Subscriber
        self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.callback)

    def callback(self, image):
        if (self.count==10): #only run with 30/self.count Hz
            
            obst_image = CompressedImage()
            obst_image.header.stamp = image.header.stamp
            obst_image.format = "jpeg"

            # pass RECTIFIED IMAGE TO DETECTOR MODULE
            obst_image.data = self.detector.process_image(rectify(rgb_from_ros(image),self.intrinsics))

            #later instead of obst_image-data more like:
            #1. EXTRACT OBSTACLES 
            #2. SEND THEM
            #3. VISUALIZE THEM!

            # publish new message
            self.publisher.publish(obst_image.data)
            self.count=1
        else:
            self.count+=1
  

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')

# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
    rospy.init_node('obst_detection_node', anonymous=False)
    obst_detection_node = ObstDetectNode()
    rospy.on_shutdown(obst_detection_node.onShutdown)
    rospy.spin()
