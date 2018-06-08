#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import TurnIDandType, FSMState, BoolStamped, LanePose, Pose2DStamped
from std_msgs.msg import Float32, Int16, Bool, String
from geometry_msgs.msg import Point
import time
import math

class UnicornIntersectionNode(object):
    def __init__(self):
        self.node_name = "Unicorn Intersection Node"

        ## setup Parameters
        self.setupParams()

        self.state = "JOYSTICK_CONTROL"

        self.active = False
        self.pos = np.array([0.0,0.0,0.0])
        self.turn_type = -1
        self.traj =np.array([[0.000,0.031,0.062,0.092,0.122,0.151,0.178,0.205,0.230,0.254,0.276,0.296,0.314,0.330,0.343,0.355,0.364,0.370,0.374,0.375,0.376,0.380,0.386,0.395,0.407,0.420,0.436,0.454,0.474,0.496,0.520,0.545,0.572,0.599,0.628,0.658,0.688,0.719,0.750],[0.000,0.001,0.005,0.011,0.020,0.032,0.045,0.061,0.079,0.099,0.121,0.145,0.170,0.197,0.224,0.253,0.283,0.313,0.344,0.375,0.406,0.437,0.467,0.497,0.526,0.553,0.580,0.605,0.629,0.651,0.671,0.689,0.705,0.718,0.730,0.739,0.745,0.749,0.750]])

        ## Subscribers
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.set_state = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_debug = rospy.Subscriber("~debugging_start", Bool, self.cbDebugging)
        self.sub_estimate = rospy.Subscriber("~estimation", Pose2DStamped, self.cbEstimation)

        ## Publisher
        self.pub_int_done = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)
        self.pub_start_est = rospy.Publisher("~start_estimation", Bool, queue_size=1)
        self.pub_pose = rospy.Publisher("~lane_pose", LanePose, queue_size=1)

        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def cbEstimation(self, msg):
        x,y,theta = msg.x, msg.y, msg.theta
        self.pos = np.array([x,y,theta])



    def cbDebugging(self, msg):
        if msg.data:
            self.active = True
            msg_bool = Bool()
            msg_bool.data = True
            self.pub_start_est.publish(msg_bool)

            self.navigationLoop()


    def cbFSMState(self, msg):
        self.state = msg.state

    def cbTurnType(self, msg):
        self.turn_type = msg.turn_type

    def navigationLoop(self):
        while self.active:
            idx_nearest = self.getNearestIdx(self.pos, self.traj)

            p1_x,p1_y = self.traj[0,idx_nearest], self.traj[1,idx_nearest]
            p2_x,p2_y = self.traj[0,idx_nearest+1], self.traj[1,idx_nearest+1]
            p1 = np.array([p1_x, p1_y])
            p2 = np.array([p2_x, p2_y])
            pc = np.array([self.pos[0], self.pos[1]])
            d_err = np.cross(p2-p1, pc-p1) / np.linalg.norm(p2-p1)
            theta_ref = -(np.arctan2(p2_x-p1_x,p2_y-p1_y)-np.pi/2)

            theta_err = theta_ref - self.pos[2]

            pose_msg = LanePose()
            pose_msg.d = d_err
            pose_msg.phi = theta_err
            pose_msg.v_ref = 0.1

            self.pub_pose.publish(pose_msg)
            rospy.sleep(0.05)


    def getNearestIdx(self, pos, traj):
        abs_min = 100
        idx_min = 0
        for i in range(0, len(traj)):
            x_t, y_t = traj[0,i], traj[1,i]
            abs_val = (pos[0]-x_t)**2 + (pos[1]-y_t)**2
            if abs_val < abs_min:
                abs_min = abs_val
                idx_min = i

        return i

    def setupParams(self):
        self.maintenance_entrance = self.setupParam("~maintenance_entrance", 0)
        self.maintenance_exit = self.setupParam("~maintenance_exit", 0)


    def updateParams(self,event):
        self.maintenance_entrance = rospy.get_param("~maintenance_entrance")
        self.maintenance_exit = rospy.get_param("~maintenance_exit")


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[UnicornIntersectionNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('unicorn_intersection_node',anonymous=False)
    unicorn_intersection_node = UnicornIntersectionNode()
    rospy.on_shutdown(unicorn_intersection_node.onShutdown)
    rospy.spin()
