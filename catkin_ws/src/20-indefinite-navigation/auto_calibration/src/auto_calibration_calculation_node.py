#!/usr/bin/env python
import rospy,copy,time,math
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped, WheelsCmdStamped
from std_msgs.msg import Int16
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import tf.transformations as tr
import numpy as np
from scipy.optimize import minimize

class AutoCalibrationCalculationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.triggered = False
        self.count_last = 0
        self.data_gathering = False
        #timestamp, omega_l, omega_r
        self.wheel_motion = np.zeros((1,3))
        #timestamp, x , y, z, roll, pitch, yaw
        self.camera_motion = np.zeros((1,7))

        #determined 9.35*math.pi by averaging Duckiebot speeds (in rad/s), this value taken from calibration files
        self.K = 27.0

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
        self.sub_record = rospy.Subscriber("~record",BoolStamped, self.cbRecord, queue_size=1)
        self.sub_duty_cycle = rospy.Subscriber("~wheels_duty_cycle",WheelsCmdStamped, self.cbDutyCycle, queue_size=1)

    #Car entered calibration calculation mode
    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING_CALC") and msg.state == "CALIBRATING_CALC":
            # Switch into CALIBRATING_CALC mode
            self.mode = msg.state
            self.triggered = True
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
            self.publishControl()
            self.data_gathering = False
            self.resample()
            self.calibration()
        self.mode = msg.state

    #Exit function for calibration calculation
    def finishCalc(self, event):
        rospy.loginfo("[%s] Calculation finished." %(self.node_name))
        done = BoolStamped()
        done.data = True
        self.pub_calc_done.publish(done)

    #extract the camera motion from apriltags
    def cbTag(self, msg):
        if self.data_gathering:
            count = 0
            tmp = np.zeros(6)
            for detection in msg.detections:
                count=count+1
                x = detection.pose.pose.pose.position.x
                y = detection.pose.pose.pose.position.x
                z = detection.pose.pose.pose.position.y
                xq = detection.pose.pose.pose.orientation.x
                yq = detection.pose.pose.pose.orientation.y
                zq = detection.pose.pose.pose.orientation.z
                wq = detection.pose.pose.pose.orientation.w
                [roll,pitch,yaw] = self.qte(wq,xq,yq,zq)
                cam_loc=np.array([x,y,z,roll,pitch,yaw])
                tmp=tmp+self.visual_odometry(cam_loc,detection.id[0])
            if count != 0:
                tmp = tmp/count
                secs = self.toSeconds(msg.header.stamp.secs,msg.header.stamp.nsecs)
                self.camera_motion = np.append(self.camera_motion,([[secs,tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5]]]),axis=0)

    #extract the wheel motion
    def cbDutyCycle(self,msg):
        if self.data_gathering:
            secs = self.toSeconds(msg.header.stamp.secs,msg.header.stamp.nsecs)
            self.wheel_motion = np.append(self.wheel_motion,([[secs,msg.vel_left*self.K,msg.vel_right*self.K]]),axis=0)

    #In calibration calculation mode, the bot shouldn't move
    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.v = 0
        car_cmd_msg.omega = 0
        car_cmd_msg.header.stamp.secs=0
        car_cmd_msg.header.stamp.nsecs=0
        self.pub_car_cmd.publish(car_cmd_msg)

    #Only test algorithm, can be deleted at the end when everything works, not used in calibration
    #Also remember to delete the corresponding topic
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

            cam_loc=np.array([0.2,0.1,0,0,20.0/180.0*3.1415926,3.1415926])
            #Homogenuous transform Tag to Cam
            tag_Rx_cam = np.array([[1,0,0],
                                   [0,np.cos(cam_loc[3]),-np.sin(cam_loc[3])],
                                   [0,np.sin(cam_loc[3]), np.cos(cam_loc[3])]])
            tag_Ry_cam = np.array([[ np.cos(cam_loc[4]),0,np.sin(cam_loc[4])],
                                   [0,1,0],
                                   [-np.sin(cam_loc[4]),0,np.cos(cam_loc[4])]])
            tag_Rz_cam = np.array([[np.cos(cam_loc[5]),-np.sin(cam_loc[5]),0],
                                   [np.sin(cam_loc[5]), np.cos(cam_loc[5]),0],
                                   [0,0,1]])
            tag_R_cam = np.dot(np.dot(tag_Rz_cam,tag_Ry_cam),tag_Rx_cam)
            tag_t_cam = np.array([[cam_loc[0]],[cam_loc[1]],[cam_loc[2]]])
            tag_T_cam = np.concatenate((np.concatenate((tag_R_cam,tag_t_cam), axis=1),np.array([[0,0,0,1]])), axis=0)
            cam_T_tag = np.linalg.inv(tag_T_cam)

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
            cam_R_bot = np.dot(np.dot(cam_Rz_bot,cam_Ry_bot),cam_Rx_bot)
            cam_t_bot = np.array([[x[4]],[x[5]],[x[6]]])
            cam_T_bot = np.concatenate((np.concatenate((cam_R_bot,cam_t_bot), axis=1),np.array([[0,0,0,1]])), axis=0)
            bot_T_cam = np.linalg.inv(cam_T_bot)

            bot_T_world = np.dot(cam_T_world,bot_T_cam)


    def cbSwitch(self, msg):
        self.active=msg.data

    def cbRecord(self, msg):
        if msg.data:
            self.data_gathering=msg.data
            self.wheel_motion = np.zeros((1,3))
            self.camera_motion = np.zeros((1,7))

#########################################################################################################
#Functions used in the calibration of the duckiebot
#########################################################################################################

    #Determines the travel of the Duckiebot using the inputs sent to the motors
    def wheels(self,x,args): #maybe done? --> Need data to check if right
        size=args.shape[1]
        #get velocity and yaw rate from kinematic model
        vel=0.5*(np.dot(x[0],args[1])+np.dot(x[1],args[2]))
        omega=-np.dot(x[0]/x[2],args[1])+np.dot(x[1]/x[3],args[2])

        #extract initial yaw angle form yaw measurement of camera
        yaw=np.zeros(size)
        yaw[0]=args[8][0]

        #calculate yaw angle for every single timestamp, using euler forward
        for i in range(1,size):
            yaw[i]=yaw[i-1]+omega[i-1]*(args[0][i]-args[0][i-1])

        x_est=0
        y_est=0
        #calculate x and y movement, using euler forward
        for i in range (0,size-1):
            x_est=x_est+(args[0][i+1]-args[0][i])*math.cos(yaw[i])*vel[i]
            y_est=y_est+(args[0][i+1]-args[0][i])*math.sin(yaw[i])*vel[i]
        return [x_est,y_est,0,0,0,yaw[size-1]]

    #Determines the travel of the Duckiebot camera from Apriltag detections
    def visual_odometry(self,cam_loc,id):
        #TODO adapt code to work with any apriltag and with camera inputs

        #Read tag location from yaml file
        tag_loc=self.tags_locations['tag'+str(id)]

        #This part can be done outside of the optimization algorithm --> save computation time
        #If more than 1 tag detected, take the average location of the N detections
        #Homogenuous transform World to Tag
        #The tags are assumed to have 0 rotation in x and y
        tag_R_world = np.array([[np.cos(tag_loc[5]),-np.sin(tag_loc[5]),0],
                                [np.sin(tag_loc[5]), np.cos(tag_loc[5]),0],
                                [0,0,1]])
        tag_t_world = np.array([[tag_loc[0]],[tag_loc[1]],[tag_loc[2]]])
        tag_T_world = np.concatenate((np.concatenate((tag_R_world,tag_t_world), axis=1),np.array([[0,0,0,1]])), axis=0)

        cam_loc=np.array([0.2,0.1,0,0,20.0/180.0*3.1415926,3.1415926])
        #Homogenuous transform Tag to Cam
        tag_Rx_cam = np.array([[1,0,0],
                               [0,np.cos(cam_loc[3]),-np.sin(cam_loc[3])],
                               [0,np.sin(cam_loc[3]), np.cos(cam_loc[3])]])
        tag_Ry_cam = np.array([[ np.cos(cam_loc[4]),0,np.sin(cam_loc[4])],
                               [0,1,0],
                               [-np.sin(cam_loc[4]),0,np.cos(cam_loc[4])]])
        tag_Rz_cam = np.array([[np.cos(cam_loc[5]),-np.sin(cam_loc[5]),0],
                               [np.sin(cam_loc[5]), np.cos(cam_loc[5]),0],
                               [0,0,1]])
        tag_R_cam = np.dot(np.dot(tag_Rz_cam,tag_Ry_cam),tag_Rx_cam)
        tag_t_cam = np.array([[cam_loc[0]],[cam_loc[1]],[cam_loc[2]]])
        tag_T_cam = np.concatenate((np.concatenate((tag_R_cam,tag_t_cam), axis=1),np.array([[0,0,0,1]])), axis=0)
        cam_T_tag = np.linalg.inv(tag_T_cam)

        cam_T_world = np.dot(tag_T_world,cam_T_tag)
        return np.array([1,2,3,4,5,6])

    #Determines the travel of the Duckiebot from the movement of its camera
    def camera(self,x,args):
        #TODO adapt code to work
        #Homogenuous transform from camera to Duckiebot 'center'
        cam_Rx_bot = np.array([[1,0,0],
                               [0,np.cos(x[7]),-np.sin(x[7])],
                               [0,np.sin(x[7]), np.cos(x[7])]])
        cam_Ry_bot = np.array([[ np.cos(x[8]),0,np.sin(x[8])],
                               [0,1,0],
                               [-np.sin(x[8]),0,np.cos(x[8])]])
        cam_Rz_bot = np.array([[np.cos(x[9]),-np.sin(x[9]),0],
                               [np.sin(x[9]), np.cos(x[9]),0],
                               [0,0,1]])
        cam_R_bot = np.dot(np.dot(cam_Rz_bot,cam_Ry_bot),cam_Rx_bot)
        cam_t_bot = np.array([[x[4]],[x[5]],[x[6]]])
        cam_T_bot = np.concatenate((np.concatenate((cam_R_bot,cam_t_bot), axis=1),np.array([[0,0,0,1]])), axis=0)
        bot_T_cam = np.linalg.inv(cam_T_bot)

        bot_T_world = np.dot(cam_T_world,bot_T_cam)
        return np.zeros(6)

    #quaternion to euler, taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles on 05.06.18
    def qte(self, w, x, y, z):
    	ysqr = y * y

    	t0 = +2.0 * (w * x + y * z)
    	t1 = +1.0 - 2.0 * (x * x + ysqr)
    	X = math.atan2(t0, t1)

    	t2 = +2.0 * (w * y - z * x)
    	t2 = +1.0 if t2 > +1.0 else t2
    	t2 = -1.0 if t2 < -1.0 else t2
    	Y = math.asin(t2)

    	t3 = +2.0 * (w * z + x * y)
    	t4 = +1.0 - 2.0 * (ysqr + z * z)
    	Z = math.atan2(t3, t4)

    	return X, Y, Z
    #euler to quaternion, adapted from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles on 05.06.18
    def etq(self,r,p,y):
        cy = math.cos(y*0.5)
        sy = math.sin(y*0.5)
        cr = math.cos(r*0.5)
        sr = math.sin(r*0.5)
        cp = math.cos(p*0.5)
        sp = math.sin(p*0.5)

        w = cy * cr * cp + sy * sr * sp
        x = cy * sr * cp - sy * cr * sp
        y = cy * cr * sp + sy * sr * cp
        z = sy * cr * cp - cy * sr * sp

        return w,x,y,z

    #The calibration calculation will come in this function
    def calibration(self):
        if not self.active:
            return
        rospy.loginfo("[%s] Calculation started." %(self.node_name))
        #Prevents multiple instances of a calibration
        if self.triggered:
            self.triggered = False
            #rospy.Timer(rospy.Duration.from_sec(5.0), self.finishCalc, oneshot=True)
            # initial guesses, still random, need to be made more precise

            # x[0] = Rl left wheel radius
            # x[1] = Rr right wheel radius
            # x[2] = bl left wheel baseline
            # x[3] = br right wheel baseline
            # x[4] = lx x-distance of camera from centerecs,nsecs,
            # x[5] = ly y-distance of camera from center
            # x[6] = lz z-distance of camera from center
            # x[7] = lr roll of camera from center
            # x[8] = lp pitch of camera from center
            # x[9] = ly yaw of camera from center

            # args[0][] = timestamp
            # args[1][] = wl wheel velocity left
            # args[2][] = wr wheel velocity right
            # args[3][] = X camera position in X
            # args[4][] = Y camera position in Y
            # args[5][] = Z camera position in Z
            # args[6][] = camera roll
            # args[7][] = camera pitch
            # args[8][] = camera yaw

            n = 10
            x0 = np.zeros(n)
            x0[0] = 0.025
            x0[1] = 0.025
            x0[2] = 0.05
            x0[3] = 0.05
            x0[4] = 0.06
            x0[5] = 0
            x0[6] = 0.075
            x0[7] = 0
            x0[8] = 0.349
            x0[9] = 0

            # bounds, need to be changed, still random
            b0 = (0.02,0.035)
            b1 = (0.02,0.035)
            b2 = (0.04,0.06)
            b3 = (0.04,0.06)
            b4 = (0.04,0.08)
            b5 = (-0.01,0.01)
            b6 = (0.06,0.09)
            b7 = (-0.174,0.174)
            b8 = (0.174,0.523)
            b9 = (-0.087,0.087)

            bnds = (b0, b1, b2, b3, b4, b5, b6, b7, b8, b9)

            #Array of camera positions and wheel velocities with corresponding timestamp
            para=np.ones((9,100))

            # optimization
            #solution = minimize(self.objective,x0,args=para, method='SLSQP',bounds=bnds)
            #x = solution.x

            self.finishCalc()

    #Objective function of the optimization algorithm
    def objective(self,x,args):
        #Weighting matrix
        Q=np.identity(6)

        #estimation by wheels
        est1=self.wheels(x,args)

        #estimation by camera
        est2=self.camera(x,args)

        return np.linalg.norm(np.dot(Q,est1-est2))

    def toSeconds(self,secs,nsecs):
        return secs + nsecs/1000000000.0

    def resample():

        return

    def setupParams(self):
        self.tags_locations = self.setupParam("~tags",0)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        return value

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
