#!/usr/bin/env python
"""
Created on Fri Apr 15 10:43:02 2016

@author: alex
"""
import rospkg
import yaml
import rospy
import time
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from duckietown_msgs.msg import AprilTags, TagDetection, TagInfo, Vector2D
import numpy as np
#import kinematic as k



class ClosedLoopTurn(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()
        
        # Publisher
        self.pub_wheels_cmd = rospy.Publisher("wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.pub_car_cmd    = rospy.Publisher("joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)
        
        # Timers
        self.loop_timer     = rospy.Timer( rospy.Duration.from_sec(10.0) , self.timedloop  )
        
        # Suscribers
        self.sub_april      = rospy.Subscriber("apriltags_postprocessing_fast_node/apriltags_out", AprilTags, self.callback, queue_size=1)
        
        # Params
        self.delay = 0.3
        self.stop_pause  = True
        
        self.speed = 0.5
        self.omega = 0.3
        
        self.target = np.array([ 0.4 , 0.15  ])
        
        rospy.loginfo("[%s] Initialized.", self.node_name)
        
        
    def timedloop(self, event):    
        """ """
        
        pass
        
        
        
    def callback(self, msg):
        """ """
        
        # Localization
        [x,y] = self.localization( msg )
        
        tag   = np.array( [x,y] )
        
        if not x == None:
        
            # Compute error
            error  = self.target - tag
            
            d = np.linalg.norm( error )
            theta = np.arctan( error[1] / error[0] )
            
            error_d_theta = np.array( [d , theta ] )

            # Bang bang            
            """
            # Select Action        
            if error[0] > 0:
                
                self.go_bck()
                rospy.loginfo("[%s] Going backward", self.node_name)
                
            else: 
                
                #self.go_fwd()
                #rospy.loginfo("[%s] Going forward", self.node_name)
                
                if y > 0:
                    
                    self.go_left()
                    rospy.loginfo("[%s] Going left", self.node_name)
                    
                else:
                    
                    self.go_right()
                    rospy.loginfo("[%s] Going right", self.node_name)
                    
            """
            
            # Prop control
            
            vel = -error[0] * 1.5
            #omg = -error[1] * 5.0
            
            #vel = -error[0] * 1.5
            omg = error_d_theta[1] * 0.5
            
            self.cmd = [  vel , omg ]
            
            self.go_cmd()
            
            print self.target, tag, error, error_d_theta , self.cmd
            
            
                
        else:
            
            self.stop()
            rospy.loginfo("[%s] No Detections", self.node_name)
        
        
    def localization(self, msg):
        """ """
        
        x = None
        y = None
        
        for detection in msg.detections:
            
            x = detection.transform.translation.x
            y = detection.transform.translation.y
            
        return [x,y]

        
        
    def go_fwd(self):
        """ """
        
        self.cmd = [  self.speed , 0 ]
        self.pub_cmd()
        if self.stop_pause :
            time.sleep( self.delay )
            self.stop()
        
    def go_bck(self):
        """ """
        
        self.cmd = [ -self.speed , 0 ]
        self.pub_cmd()
        if self.stop_pause :
            time.sleep( self.delay )
            self.stop()
        
    def go_right(self):
        """ """
        
        self.cmd = [ self.speed , -self.omega ]
        self.pub_cmd()
        if self.stop_pause :
            time.sleep( self.delay )
            self.stop()
        
    def go_left(self):
        """ """
        
        self.cmd = [ self.speed , self.omega ]
        self.pub_cmd()
        if self.stop_pause :
            time.sleep( self.delay )
            self.stop()
        
    def go_cmd(self):
        """ """
        
        self.pub_cmd()
        if self.stop_pause :
            time.sleep( self.delay )
            self.stop()
        
        
        
    def pub_cmd(self):
        """ """   
        """
        # direct to wheels
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.vel_right = self.cmd[0]
        msg_wheels_cmd.vel_left  = self.cmd[1]
        self.pub_wheels_cmd.publish(msg_wheels_cmd)
        """
        # to inverse kinematic node
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v       = self.cmd[0]
        car_cmd_msg.omega   = self.cmd[1]
        self.pub_car_cmd.publish(car_cmd_msg)
        
        
        
    def stop(self):
        """ """   
        """
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.vel_right = 0
        msg_wheels_cmd.vel_left  = 0
        self.pub_wheels_cmd.publish( msg_wheels_cmd )
        """
        
        # to inverse kinematic node
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v       = 0
        car_cmd_msg.omega   = 0
        self.pub_car_cmd.publish(car_cmd_msg)
        
        
        
        
        

if __name__ == '__main__': 
    rospy.init_node('ClosedLoopTurn',anonymous=False)
    node = ClosedLoopTurn()
    rospy.spin()