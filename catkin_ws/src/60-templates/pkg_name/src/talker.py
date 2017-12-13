#!/usr/bin/env python
import rospy
from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg
#from sensor_msgs.msg import CameraInfo, CompressedImage, Image
#from duckietown_utils.jpg import image_cv_from_jpg
#from cv_bridge import CvBridge  
from sys import argv

class Talker(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        #self.pub_img = rospy.Publisher("/talker/filter/compressed",Image,queue_size=1)
        # Setup subscriber ,"" has to be equal to launch file channel
        self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
		# Read parameters
        print("Hi a")
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
       	self.filter=self.setupParameter("~filter", "x")
       	filter_st=self.filter.value
       	print("filter_st " + filter_st)
        # change inside "" will be displayed at the beginning in terminal
        # Create a timer that calls the cbTimer function every 1.0 second
        print("hi b")
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
    	print("hi1")
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTopic(self,msg):
        rospy.loginfo("[%s] %s" %(self.node_name,msg.data))
        print("Hi2") #not in rospy loginfo
        s = "I heard: %s" % (msg.data)
    	rospy.loginfo("[%s] %s" %(self.node_name, s))

    def cbTimer(self,event):
    	print("hi3")
        singer = HelloGoodbye()
        # Simulate hearing something
        msg = String()
        msg.data = singer.sing("duckietown duck")
        self.pub_topic_a.publish(msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('talker', anonymous=False)

    # Create the NodeName object
    node = Talker()

    #filters = argv[3]
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()