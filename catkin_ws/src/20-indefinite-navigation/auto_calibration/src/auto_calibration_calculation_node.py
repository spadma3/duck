#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped
from std_msgs.msg import Int16
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Pose
import copy
import time
import tf.transformations as tr

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

    #Decide which commands should be sent to wheels in calibration mode
    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = 0
        car_cmd_msg.omega = 0
        self.pub_car_cmd.publish(car_cmd_msg)

    def cbTest(self,msg):
        if msg.data:
            tag=Pose()
            tag.position.x = 1
            tag.position.y = 0.5
            tag.position.z = 0.05
            tag.orientation.x = 0
            tag.orientation.y = 0
            tag.orientation.z = 1
            tag.orientation.w = 0
            tag_t_world = tr.translation_matrix((tag.position.x,tag.position.y,tag.position.z))
            tag_R_world = tr.quaternion_matrix((tag.orientation.w,tag.orientation.x,tag.orientation.y,tag.orientation.z))
            tag_T_world = tr.concatenate_matrices(tag_t_world,tag_R_world)

            cam=Pose()
            cam.position.x = 0.2
            cam.position.y = 0.1
            cam.position.z = 0
            cam.orientation.x = 0.173643
            cam.orientation.y = 0
            cam.orientation.z = 0.984808
            cam.orientation.w = 0
            tag_t_cam= tr.translation_matrix((cam.position.x,cam.position.y,cam.position.z))
            tag_R_cam = tr.quaternion_matrix((cam.orientation.w,cam.orientation.x,cam.orientation.y,cam.orientation.z))
            tag_T_cam = tr.concatenate_matrices(tag_t_cam,tag_R_cam)
            cam_T_tag = tr.inverse_matrix(tag_T_cam)

            cam_T_world = tr.concatenate_matrices(cam_T_tag,tag_T_world)

            cam_abs=Pose()
            cam_abs.position = tr.translation_from_matrix(cam_T_world)
            (roll,pitch,yaw) = tr.euler_from_quaternion(tr.quaternion_from_matrix(cam_T_world))

            tag_loc=self.tags_locations['tag300']
            rospy.loginfo("test %s" %(tag_loc[0]))

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
