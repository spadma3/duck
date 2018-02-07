#!/usr/bin/env python
import rospy
import math
from duckietown_msgs.msg import Twist2DStamped, LanePose
from geometry_msgs.msg import PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

class lane_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = None

        self.pub_counter = 0

        # Setup parameters
        self.setGains()

        # Publicaiton
        self.pub_car_cmd = rospy.Publisher("/lex/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_tag_detector = rospy.Subscriber("/lex/tag_detections_pose", PoseArray, self.cbTag, queue_size=1)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        # timer
        # self.gains_timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.getGains_event)

        self.drive_d = 0
        self.drive_phi = 0
        self.drive_upon = rospy.Time()

        self.drive_timer = rospy.Timer(rospy.Duration.from_sec(0.02), self.drive)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def cbTag(self, tag_msg):
        poses = tag_msg.poses

        if len(poses) == 0:
            return

        p_x = poses[0].position.x
        p_y = poses[0].position.y
        p_z = poses[0].position.z
        orientation_list = [poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z, poses[0].orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


        if tag_msg.header.stamp < self.drive_upon:
            return

        d = p_x
        phi = math.atan(p_z/p_x)
        phi = 0

        self.driveFor(d, phi, 1)

        rospy.loginfo("d: " + str(d) + "  phi: " + str(phi) )

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def setGains(self):
        v_bar = 0.23 # nominal speed, 0.5m/s
        k_theta = -2.0 * 0.2
        k_d = - 7
        theta_thres = math.pi / 6
        d_thres = math.fabs(k_theta / k_d) * theta_thres
        d_offset = 0.0

        self.v_bar = self.setupParameter("~v_bar",v_bar) # Linear velocity
        # FIXME: AC aug'17: are these inverted?
        self.k_d = self.setupParameter("~k_d",k_theta) # P gain for theta
        self.k_theta = self.setupParameter("~k_theta",k_d) # P gain for d
        self.d_thres = self.setupParameter("~d_thres",theta_thres) # Cap for error in d
        self.theta_thres = self.setupParameter("~theta_thres",d_thres) # Maximum desire theta
        self.d_offset = self.setupParameter("~d_offset",d_offset) # a configurable offset from the lane position


        self.v_bar = 0.15
        self.k_theta = k_theta
        self.k_d = k_d

    def driveFor(self, d, phi, secs):
        self.drive_d = d
        self.drive_phi = phi
        timestamp_now = rospy.Time.now()



        self.drive_upon = timestamp_now + rospy.Duration(secs)


    def drive(self, event):
        d = 0
        phi = 0
        timestamp_now = rospy.Time.now()
        if self.drive_upon > timestamp_now:
            d = self.drive_d
            phi = self.drive_phi
            self.cbPose(d, phi)


    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")

        params_old = (self.v_bar,self.k_d,self.k_theta,self.d_thres,self.theta_thres, self.d_offset)
        params_new = (v_bar,k_d,k_theta,d_thres,theta_thres, d_offset)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." %(self.node_name))
            rospy.loginfo("old gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_old))
            rospy.loginfo("new gains, v_var %f, k_d %f, k_theta %f, theta_thres %f, d_thres %f, d_offset %f" %(params_new))
            self.v_bar = v_bar
            self.k_d = k_d
            self.k_theta = k_theta
            self.d_thres = d_thres
            self.theta_thres = theta_thres
            self.d_offset = d_offset


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." %self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()

        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)
        rospy.sleep(0.5) #To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):

        #wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        #speed_gain = 1.0
        #steer_gain = 0.5
        #vel_left = (speed_gain*speed - steer_gain*steering)
        #vel_right = (speed_gain*speed + steer_gain*steering)
        #wheels_cmd_msg.vel_left = np.clip(vel_left,-1.0,1.0)
        #wheels_cmd_msg.vel_right = np.clip(vel_right,-1.0,1.0)

        self.pub_car_cmd.publish(car_cmd_msg)
        #self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def cbPose(self, d, phi ):


        # # Calculating the delay image processing took
        # timestamp_now = rospy.Time.now()
        # image_delay_stamp = timestamp_now - self.lane_reading.header.stamp
        #
        # # delay from taking the image until now in seconds
        # image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs/1e9

        cross_track_err = d
        heading_err = phi

        car_control_msg = Twist2DStamped()
        # car_control_msg.header = lane_pose_msg.header
        car_control_msg.v = self.v_bar #*self.speed_gain #Left stick V-axis. Up is positive

        if math.fabs(cross_track_err) > self.d_thres:
            cross_track_err = cross_track_err / math.fabs(cross_track_err) * self.d_thres
        car_control_msg.omega =  self.k_d * cross_track_err + self.k_theta * heading_err #*self.steer_gain #Right stick H-axis. Right is negative

        # controller mapping issue
        # car_control_msg.steering = -car_control_msg.steering
        # print "controls: speed %f, steering %f" % (car_control_msg.speed, car_control_msg.steering)
        # self.pub_.publish(car_control_msg)
        self.publishCmd(car_control_msg)

        # debuging
        # self.pub_counter += 1
        # if self.pub_counter % 50 == 0:
        #     self.pub_counter = 1
        #     print "lane_controller publish"
        #     print car_control_msg

if __name__ == "__main__":
    rospy.init_node("lane_controller",anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
