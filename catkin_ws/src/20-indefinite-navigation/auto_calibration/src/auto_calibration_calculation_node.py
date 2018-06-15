#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped
from std_msgs.msg import Int16
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import copy
import time
import tf.transformations as tr
import numpy as np

class AutoCalibrationCalculationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.triggered = False
        self.count_last = 0

        #Node active?
        self.active = False

        #Parameters
        self.setupParams()

        #Publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.pub_calc_done = rospy.Publisher("~calibration_calculation_stop",BoolStamped, queue_size=1)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_tags = rospy.Subscriber("~tag",AprilTagDetectionArray, self.cbTag, queue_size=10)
        self.sub_test = rospy.Subscriber("~test",BoolStamped, self.cbTest, queue_size=1)

    #Car entered calibration calculation mode
    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING_CALC") and msg.state == "CALIBRATING_CALC":
            # Switch into CALIBRATING_CALC mode
            self.mode = msg.state
            self.triggered = True
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
            self.calibration()
            self.publishControl()
        self.mode = msg.state

    #The calibration calculation will come in this function
    def calibration(self):
        rospy.loginfo("[%s] Calculation started." %(self.node_name))
        if self.triggered:
            rospy.Timer(rospy.Duration.from_sec(5.0), self.finishCalc, oneshot=True)
            self.triggered = False

    #Exit function for calibration calculation
    def finishCalc(self, event):
        rospy.loginfo("[%s] Calculation finished." %(self.node_name))
        done = BoolStamped()
        done.data = True
        self.pub_calc_done.publish(done)

    def cbTag(self, msg):
        count = 0
        for detection in msg.detections:
            count=count+1
        if count!=0 and self.count_last!=0:
            rospy.loginfo("[%s] Calculation started %s - %s" %(self.node_name, count, self.count_last))
        self.count_last = count

    #In calibration calculation mode, the bot shouldn't move
    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = 0
        car_cmd_msg.omega = 0
        self.pub_car_cmd.publish(car_cmd_msg)

    def cbTest(self,msg):
        if msg.data:
            #Read tag location from yaml file
            tag_loc=self.tags_locations['tag300']

            #This part can be done outside of the optimization algorithm --> save computation time
            #If more than 1 tag detected, take the average location of the N detections
            #Homogenuous transform World to Tag
            #The tags are assumed to have 0 rotation in x and y
            tag_R_world = np.array([[np.cos(tag_loc[5]),-np.sin(tag_loc[5]),0],
                                    [np.sin(tag_loc[5]), np.cos(tag_loc[5]),0],
                                    [0,0,1]])
            tag_t_world = np.array([[tag_loc[0]],[tag_loc[1]],[tag_loc[2]]])
            tag_T_world = np.concatenate((np.concatenate((tag_R_world,tag_t_world), axis=1),np.array([[0,0,0,1]])), axis=0)

            cam_loc=np.array([0.2,0.1,0,0,20/180*3.1415926,3.1415926])
            #Homogenuous transform Tag to Cam
            #something wrong between lines 96 and 110 TODO
            tag_Rx_cam = np.array([[1,0,0],
                                   [0,np.cos(cam_loc[3]),-np.sin(cam_loc[3])],
                                   [0,np.sin(cam_loc[3]), np.cos(cam_loc[3])]])
            tag_Ry_cam = np.array([[ np.cos(cam_loc[4]),0,np.sin(cam_loc[4])],
                                   [0,1,0],
                                   [-np.sin(cam_loc[4]),0,np.cos(cam_loc[4])]])
            tag_Rz_cam = np.array([[np.cos(cam_loc[5]),-np.sin(cam_loc[5]),0],
                                   [np.sin(cam_loc[5]), np.cos(cam_loc[5]),0],
                                   [0,0,1]])
            tag_R_cam = np.dot(np.dot(tag_Rx_cam,tag_Ry_cam),tag_Rz_cam)
            tag_t_cam = np.array([[cam_loc[0]],[cam_loc[1]],[cam_loc[2]]])
            tag_T_cam = np.concatenate((np.concatenate((tag_R_cam,tag_t_cam), axis=1),np.array([[0,0,0,1]])), axis=0)
            cam_T_tag = np.linalg.inv(tag_T_cam)

            rospy.loginfo("test %s" %(cam_T_tag))

            cam_T_world = np.dot(tag_T_world,cam_T_tag)

            x=np.array([0,0,0,0,0,0,0,0,0,0])
            #to be put into optimization algorithm, x is the parameter vector
            cam_Rx_bot = np.array([[1,0,0],
                                   [0,np.cos(x[7]),-np.sin(x[7])],
                                   [0,np.sin(x[7]), np.cos(x[7])]])
            cam_Ry_bot = np.array([[ np.cos(x[8]),0,np.sin(x[8])],
                                   [0,1,0],
                                   [-np.sin(x[8]),0,np.cos(x[8])]])
            cam_Rz_bot = np.array([[np.cos(x[9]),-np.sin(x[9]),0],
                                   [np.sin(x[9]), np.cos(x[9]),0],
                                   [0,0,1]])
            cam_R_bot = np.dot(np.dot(cam_Rx_bot,cam_Ry_bot),cam_Rz_bot)
            cam_t_bot = np.array([[x[4]],[x[5]],[x[6]]])
            cam_T_bot = np.concatenate((np.concatenate((cam_R_bot,cam_t_bot), axis=1),np.array([[0,0,0,1]])), axis=0)
            bot_T_cam = np.linalg.inv(cam_T_bot)

            bot_T_world = np.dot(cam_T_world,bot_T_cam)

            #cam_abs.position = tr.translation_from_matrix(cam_T_world)
            #(roll,pitch,yaw) = tr.euler_from_quaternion(tr.quaternion_from_matrix(cam_T_world))

    def setupParams(self):
        self.tags_locations = self.setupParam("~tags",0)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        return value

    def cbSwitch(self, msg):
        self.active=msg.data

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('auto_calibration_calculation_node', anonymous=False)

    # Create the NodeName object
    node = AutoCalibrationCalculationNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)

    # Keep it spinning to keep the node alive
    rospy.spin()
