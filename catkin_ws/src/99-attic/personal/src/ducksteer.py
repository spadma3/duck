#!/usr/bin/env python
import rospy
from julien_test.util import HelloGoodbye #Imports module. Not limited to modules in this pkg.
import std_msgs.msg
from duckietown_msgs.msg import LanePose, WheelsCmdStamped
from std_msgs.msg import String
import random
import math
import time

class DuckSteer(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.list_d = [0,0,0,0,0,0,0,0,0]
        self.list_phi = [0,0,0,0,0,0,0,0,0]
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers TODO: Configure node location for all bots, not sure how this works yet ;)
        self.pub_wheels_cmd = rospy.Publisher("/lex/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        # Setup subscriber
        self.sub_lane_pose = rospy.Subscriber("/lex/lane_filter_node/lane_pose", LanePose, self.cbPose, queue_size=1)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.05)
        # Create a timer
        #self.timer = rospy.Timer(rospy.Duration(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTopic(self,msg):

        rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

    def cbPose(self, lane_pose_msg):
        self.lane_reading = lane_pose_msg
        #rospy.loginfo(self.lane_reading.d)
        self.phi = self.lane_reading.phi
        self.d = self.lane_reading.d #3*(self.lane_reading.d -0.125)

        self.list_d.append(self.d)
        self.list_d.pop(0)
        self.list_phi.append(self.phi)
        self.list_phi.pop(0)
        avg_d = sum(self.list_d) / float(len(self.list_d))
        avg_phi = sum(self.list_phi) / float(len(self.list_phi))
        rospy.loginfo("D:       " + str(avg_d))
        rospy.loginfo("Phi:     " + str(avg_phi))
        # Distance between wheels
        b = 0.1
        # Standard velocity
        gain = 0.5
        v0g = 0.2
        v0 = 0.145/0.3*v0g
        gain_p = 2
        #self.s = b/2/v0 * avg_d*gain - b*avg_phi*gain
        self.s = b/2/v0 * self.d*gain - b*self.phi*gain_p



        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        wcs = WheelsCmdStamped()
        wcs.header = header
        wcs.vel_left=v0g - self.s/2
        wcs.vel_right=v0g + self.s/2
        self.pub_wheels_cmd.publish(wcs)


    def cbTimer(self,event):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        wcs = WheelsCmdStamped()
        wcs.header = header
        wcs.vel_left=0.2+0.15*math.cos(time.time()*2*math.pi/2)
        wcs.vel_right=0.2+0.05*math.sin(time.time()*2*math.pi/2)
        self.pub_wheels_cmd.publish(wcs)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        wcs = WheelsCmdStamped()
        wcs.header = h
        wcs.vel_left=0
        wcs.vel_right=0
        self.pub_wheels_cmd.publish(wcs)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('ducksteer', anonymous=False)

    # Create the NodeName object
    node = DuckSteer()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
