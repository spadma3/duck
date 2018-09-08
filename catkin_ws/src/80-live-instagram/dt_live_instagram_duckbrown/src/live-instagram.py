#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import duckietown_utils as dt
from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import CompressedImage
class Insta(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        # Read parameters
        self.topic = self.setupParameter("~topic", "/duckbrown/camera_node/image")
        self.filter = self.setupParameter("~filter","greyscale")
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Setup publishers
        self.pub_topic_a = rospy.Publisher(self.topic+"filter/compressed", CompressedImage, queue_size=1)
        # Setup subscriber
        self.sub_topic_b = rospy.Subscriber(self.topic+"compressed", CompressedImage, self.cbFilter)


        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

    def cbFilter(self,msg):
        filter = self.setupParameter("~filter")
        image = dt.rgb_from_ros(msg)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if filter == 'flip-vertical':
            image = np.fliplr(image)
        if filter == 'flip-horizontal':
            image = np.flipud(image)
        if filter == 'greyscale':
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if filter == 'sepia':
            image_copy = np.empty(image.shape, 'float')
            image = cv2.transform(image, M_Sepia, image_copy)
        new_msg = dt.d8_compressed_image_from_cv_image(image)
        self.pub_topic_a.publish(new_msg)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('dt_live_instagram_duckbrown_node', anonymous=False)

    # Create the NodeName object
    node = Insta()
    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
