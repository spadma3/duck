#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
import time
import threading

### note you need to change the name of the robot to yours here
from obst_avoid.detector import Detector
from obst_avoid.visualizer import Visualizer
from duckietown_utils import get_base_name, rgb_from_ros, rectify, load_camera_intrinsics, d8_compressed_image_from_cv_image

class ObstDetectNode(object):
    """
    Obstacle Detection Node
    """
    def __init__(self):
        self.node_name = "Obstacle Detecion Node"
        robot_name = rospy.get_param("~veh", "")
        self.show_marker = (rospy.get_param("~show_marker", ""))
        self.show_image = (rospy.get_param("~show_image", ""))
        
        self.r = rospy.Rate(3) # Rate in Hz
        self.thread_lock = threading.Lock()

        self.detector = Detector(robot_name=robot_name)

        # Load camera calibration parameters
	self.intrinsics = load_camera_intrinsics(robot_name)

        # Create a Publisher
        self.pub_topic_arr = '/{}/obst_detect/posearray'.format(robot_name)
        self.publisher_arr = rospy.Publisher(self.pub_topic_arr, PoseArray, queue_size=1)

        if (self.show_marker or self.show_image):
                self.visualizer = Visualizer(robot_name=robot_name)

        if (self.show_marker):
                self.pub_topic_marker = '/{}/obst_detect/visualize_obstacles'.format(robot_name)
                self.publisher_marker = rospy.Publisher(self.pub_topic_marker, MarkerArray, queue_size=1)
                print "show_marker is active: marker will be published as /veh/obst_detect/visualize_obstacles"

        if (self.show_image):
                self.pub_topic_img = '/{}/obst_detect/image_cropped/compressed'.format(robot_name)
                self.publisher_img = rospy.Publisher(self.pub_topic_img, CompressedImage, queue_size=1)
                print "show_image is active: image will be published as /veh/obst_detect/image_cropped/compressed"

        # Create a Subscriber
        self.sub_topic = '/{}/camera_node/image/compressed'.format(robot_name)
        self.subscriber = rospy.Subscriber(self.sub_topic, CompressedImage, self.callback_img,queue_size=1, buff_size=2**24)
        #buff size to approximately close to 2^24 such that always most recent pic is taken
        #essentail 

    def callback_img(self, image):
        thread = threading.Thread(target=self.callback,args=(image,))
        thread.setDaemon(True)
        thread.start()



    def callback(self, image):
        if not self.thread_lock.acquire(False):
            return

        start = time.time()
        obst_list = PoseArray()
        marker_list = MarkerArray()
    
        # pass RECTIFIED IMAGE TO DETECTOR MODULE
        #1. EXTRACT OBSTACLES and return the pose array
        obst_list = self.detector.process_image(rectify(rgb_from_ros(image),self.intrinsics))

        obst_list.header.stamp = image.header.stamp #for synchronization
        #interessant um zu schauen ob stau oder nicht!!!!
        #print image.header.stamp.to_sec()
        self.publisher_arr.publish(obst_list)
        #EXPLANATION: (x,y) is world coordinates of obstacle, z is radius of obstacle
        #QUATERNION HAS NO MEANING!!!!    

        #3. VISUALIZE POSE ARRAY IN TF
        if (self.show_marker):
                marker_list = self.visualizer.visualize_marker(obst_list)
                self.publisher_marker.publish(marker_list)


        #4. EVENTUALLY DISPLAY !!!!CROPPED!!!!!! IMAGE
        if (self.show_image):
                obst_image = CompressedImage()
                obst_image.header.stamp = image.header.stamp
                obst_image.format = "jpeg"
                obst_image.data = self.visualizer.visualize_image(rectify(rgb_from_ros(image),self.intrinsics),obst_list)
                #here i want to display cropped image
                image=rgb_from_ros(obst_image.data)
                obst_image.data = d8_compressed_image_from_cv_image(image[self.detector.crop:,:,::-1])
                #THIS part only to visualize the cropped version -> somehow a little inefficient but keeps
                #the visualizer.py modular!!!
                self.publisher_img.publish(obst_image.data)

        end = time.time()
        print "GOING THROUGH TOOK: s"
        print(end - start)
        self.r.sleep()
        self.thread_lock.release()

    def onShutdown(self):
        rospy.loginfo('Shutting down Obstacle Detection, back to unsafe mode')

# MEINER MEINUNG NACH HIER DANN WARSCH 2.NODE AUCH NOCH REIN WO DANN DIE OBST AVOIDANCE GEMACHT WIRD ODER SO

if __name__ == '__main__':
    rospy.init_node('obst_detection_node', anonymous=False)
    obst_detection_node = ObstDetectNode()
    rospy.on_shutdown(obst_detection_node.onShutdown)
    rospy.spin()
