#!/usr/bin/env python
import rospy,math # ,copy,time
from duckietown_msgs.msg import FSMState, BoolStamped, Twist2DStamped, WheelsCmdStamped
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
import numpy as np
np.set_printoptions(threshold=np.nan)
from scipy.optimize import minimize

class AutoCalibrationCalculationNode(object):

    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.triggered = False
        self.count_last = 0
        self.data_gathering = False #ONLY for testing purposes, default is FALSE
        # Number of steps to integrate omega for wheelspeeds
        self.intersampling = 100
        self.intersampling_f = 100.0

        #timestamp, omega_l, omega_r
        #self.wheel_motion = np.zeros((1,3))
        #self.wheel_motion = np.array([[0,0.4,0.4],[0.1,0.4,0.4],[0.2,0.4,0.4],[0.3,0.4,0.4],[0.4,0.4,0.4],[0.5,0.4,0.4],[0.6,0.4,0.4],[0.7,0.4,0.4],[0.8,0.4,0.4],[0.9,0.4,0.4],[1,0.4,0.4],[1.1,0.4,0.4],[1.2,0.4,0.4],[1.3,0.4,0.4],[1.4,0.4,0.4],[1.5,0.4,0.4],[1.6,0.4,0.4],[1.7,0.4,0.4],[1.8,0.4,0.4],[1.9,0.4,0.4],[2,0.5,0.3],[2.1,0.5,0.3],[2.2,0.5,0.3],[2.3,0.5,0.3],[2.4,0.5,0.3],[2.5,0.5,0.3],[2.6,0.5,0.3],[2.7,0.5,0.3],[2.8,0.5,0.3],[2.9,0.5,0.3],[3,0.5,0.3],[3.1,0.5,0.3],[3.2,0.5,0.3],[3.3,0.5,0.3],[3.4,0.5,0.3],[3.5,0.5,0.3],[3.6,0.5,0.3],[3.7,0.5,0.3],[3.8,0.5,0.3],[3.9,0.5,0.3],[4,0.5,0.3],[4.1,0.5,0.3],[4.2,0.5,0.3],[4.3,0.5,0.3],[4.4,0.5,0.3],[4.5,0.5,0.3],[4.6,0.5,0.3],[4.7,0.5,0.3],[4.8,0.5,0.3],[4.9,0.5,0.3],[5,0.3,0.5],[5.1,0.3,0.5],[5.2,0.3,0.5],[5.3,0.3,0.5],[5.4,0.3,0.5],[5.5,0.3,0.5],[5.6,0.3,0.5],[5.7,0.3,0.5],[5.8,0.3,0.5],[5.9,0.3,0.5],[6,0.3,0.5],[6.1,0.3,0.5],[6.2,0.3,0.5],[6.3,0.3,0.5],[6.4,0.3,0.5],[6.5,0.3,0.5],[6.6,0.3,0.5],[6.7,0.3,0.5],[6.8,0.3,0.5],[6.9,0.3,0.5],[7,0.3,0.5],[7.1,0.3,0.5],[7.2,0.3,0.5],[7.3,0.3,0.5],[7.4,0.3,0.5],[7.5,0.3,0.5],[7.6,0.3,0.5],[7.7,0.3,0.5],[7.8,0.3,0.5],[7.9,0.3,0.5],[8,0.4,0.4],[8.1,0.4,0.4],[8.2,0.4,0.4],[8.3,0.4,0.4],[8.4,0.4,0.4],[8.5,0.4,0.4],[8.6,0.4,0.4],[8.7,0.4,0.4],[8.8,0.4,0.4],[8.9,0.4,0.4],[9,0.4,0.4],[9.1,0.4,0.4],[9.2,0.4,0.4],[9.3,0.4,0.4],[9.4,0.4,0.4],[9.5,0.4,0.4],[9.6,0.4,0.4],[9.7,0.4,0.4],[9.8,0.4,0.4],[9.9,0.4,0.4],[10,0.4,0.4]])


        self.wheel_motion =np.array([[0,0.39643,0.39953],[0.1,0.39611,0.40318],[0.2,0.39757,0.39675],[0.3,0.39846,0.39965],[0.4,0.39681,0.40396],[0.5,0.39866,0.39838],[0.6,0.3965,0.39839],[0.7,0.39637,0.40004],[0.8,0.40209,0.40105],[0.9,0.39672,0.39665],[1,0.40222,0.40324],[1.1,0.40027,0.39687],[1.2,0.40261,0.3987],[1.3,0.39835,0.40197],[1.4,0.39608,0.39639],[1.5,0.40134,0.40083],[1.6,0.40021,0.40184],[1.7,0.40166,0.40225],[1.8,0.3983,0.40154],[1.9,0.40045,0.39917],[2,0.49562,0.30168],[2.1,0.49838,0.30065],[2.2,0.50241,0.29763],[2.3,0.49628,0.3003],[2.4,0.49985,0.30234],[2.5,0.50299,0.30141],[2.6,0.49551,0.29744],[2.7,0.49589,0.30179],[2.8,0.50443,0.3011],[2.9,0.49632,0.30134],[3,0.4961,0.2977],[3.1,0.50141,0.29897],[3.2,0.50154,0.30149],[3.3,0.50083,0.30144],[3.4,0.49735,0.30141],[3.5,0.50471,0.3022],[3.6,0.49586,0.2992],[3.7,0.49869,0.30111],[3.8,0.50098,0.30174],[3.9,0.49868,0.29824],[4,0.49587,0.30163],[4.1,0.49706,0.29933],[4.2,0.50052,0.29837],[4.3,0.50142,0.29991],[4.4,0.49652,0.30169],[4.5,0.49601,0.29876],[4.6,0.49737,0.30019],[4.7,0.49591,0.29943],[4.8,0.49605,0.29767],[4.9,0.50284,0.29875],[5,0.30062,0.50464],[5.1,0.29959,0.50195],[5.2,0.30155,0.49933],[5.3,0.30093,0.4961],[5.4,0.3026,0.49687],[5.5,0.2986,0.50298],[5.6,0.29993,0.50269],[5.7,0.29938,0.49773],[5.8,0.29722,0.50173],[5.9,0.29958,0.49952],[6,0.30066,0.49559],[6.1,0.29889,0.50273],[6.2,0.30118,0.49625],[6.3,0.29778,0.49592],[6.4,0.29705,0.49923],[6.5,0.30093,0.50223],[6.6,0.30019,0.49609],[6.7,0.30079,0.49626],[6.8,0.29781,0.49599],[6.9,0.29785,0.49668],[7,0.29818,0.49817],[7.1,0.2989,0.49718],[7.2,0.29851,0.50393],[7.3,0.30122,0.50056],[7.4,0.29811,0.49712],[7.5,0.29746,0.50414],[7.6,0.30124,0.50058],[7.7,0.29888,0.49666],[7.8,0.30073,0.50488],[7.9,0.29802,0.49758],[8,0.39917,0.39659],[8.1,0.40147,0.39922],[8.2,0.40386,0.39922],[8.3,0.40097,0.39723],[8.4,0.39905,0.39729],[8.5,0.40206,0.40297],[8.6,0.39881,0.40148],[8.7,0.39835,0.40025],[8.8,0.40266,0.40078],[8.9,0.39868,0.39839],[9,0.39962,0.39938],[9.1,0.39888,0.40047],[9.2,0.40194,0.39939],[9.3,0.39943,0.397],[9.4,0.3962,0.39832],[9.5,0.39854,0.40123],[9.6,0.40366,0.40349],[9.7,0.39966,0.39792],[9.8,0.40211,0.40207],[9.9,0.40193,0.40195],[10,0.39685,0.40145]])

        #timestamp, x , y, z, roll, pitch, yaw
        #self.camera_motion = np.zeros((1,7))


        #self.camera_motion = np.array([[0,0.055,0,0.1,0,0.349,0],[0.5,0.23249,0,0.1,0,0.349,0],[1,0.41069,0,0.1,0,0.349,0],[1.5,0.58889,0,0.1,0,0.349,0],[2,0.76709,0,0.1,0,0.349,0],[2.5,0.90262,-0.11637,0.1,0,0.349,-0.8910],[3,0.89717,-0.29505,0.1,0,0.349,-1.7820],[3.5,0.75478,-0.40313,0.1,0,0.349,-2.6730],[4,0.58121,-0.36034,0.1,0,0.349,2.7192],[4.5,0.50538,-0.19846,0.1,0,0.349,1.8282],[5,0.58361,-0.037724,0.1,0,0.349,0.9372],[5.5,0.57022,0.14028,0.1,0,0.349,1.8282],[6,0.423,0.24168,0.1,0,0.349,2.7192],[6.5,0.25159,0.19092,0.1,0,0.349,-2.6730],[7,0.18332,0.025711,0.1,0,0.349,-1.7820],[7.5,0.26888,-0.13124,0.1,0,0.349,-0.8910],[8,0.44473,-0.16337,0.1,0,0.349,0],[8.5,0.62293,-0.16318,0.1,0,0.349,0],[9,0.80113,-0.16318,0.1,0,0.349,0],[9.5,0.97933,-0.16318,0.1,0,0.349,0],[10,1.1575,-0.16318,0.1,0,0.349,0]])

        self.camera_motion =np.array([[0,0.05496,0,0.099197,0,0.34673,0],[0.5,0.23326,0,0.10003,0,0.34658,0],[1,0.41102,0,0.099073,0,0.35074,0],[1.5,0.58918,0,0.10009,0,0.34841,0],[2,0.76333,0,0.10085,0,0.35202,0],[2.5,0.91022,-0.11706,0.10015,0,0.34731,-0.89549],[3,0.8923,-0.29248,0.10053,0,0.3505,-1.7871],[3.5,0.75356,-0.40225,0.10063,0,0.3512,-2.6885],[4,0.58531,-0.36038,0.10027,0,0.34861,2.6953],[4.5,0.50909,-0.19898,0.09971,0,0.34707,1.8338],[5,0.58483,-0.037639,0.099284,0,0.34845,0.93128],[5.5,0.5728,0.13992,0.10068,0,0.3495,1.8164],[6,0.42687,0.24054,0.10085,0,0.34812,2.6968],[6.5,0.2523,0.1897,0.09909,0,0.34794,-2.6816],[7,0.18289,0.025777,0.099043,0,0.3511,-1.7908],[7.5,0.27056,-0.13094,0.10023,0,0.34921,-0.88699],[8,0.44249,-0.16321,0.099455,0,0.35239,0],[8.5,0.62337,-0.16183,0.1006,0,0.34598,0],[9,0.79341,-0.16378,0.10057,0,0.35169,0],[9.5,0.9818,-0.16199,0.099436,0,0.3458,0],[10,1.1602,-0.16461,0.099709,0,0.35238,0]])

        #determined 9.35*math.pi by averaging Duckiebot speeds (in rad/s), this value taken from calibration files
        self.K = 27.0

        #Node active?
        self.active = True

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
        self.sub_record = rospy.Subscriber("~record",BoolStamped, self.cbRecord, queue_size=1)
        self.sub_duty_cycle = rospy.Subscriber("~wheels_duty_cycle",WheelsCmdStamped, self.cbDutyCycle, queue_size=1)

    #Car entered calibration calculation mode
    def cbFSMState(self,msg):
        if (not self.mode == "CALIBRATING_CALC") and msg.state == "CALIBRATING_CALC":
            # Switch into CALIBRATING_CALC mode
            self.data_gathering = False
            self.mode = msg.state
            self.triggered = True
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
            #self.publishControl()
            self.resample()
            self.calibration()
        self.mode = msg.state

    #Exit function for calibration calculation
    def finishCalc(self):
        rospy.loginfo("[%s] Calculation finished." %(self.node_name))
        done = BoolStamped()
        done.data = True
        self.pub_calc_done.publish(done)

    #extract the camera motion from apriltags
    def cbTag(self, msg):
        if self.data_gathering:
            count = 0
            tmp = np.zeros(9)
            tmp_6 = np.zeros(6)
            for detection in msg.detections:
                count=count+1
                #Coordinate transformation
                x = detection.pose.pose.pose.position.z
                y = -detection.pose.pose.pose.position.x
                z = -detection.pose.pose.pose.position.y
                xq = detection.pose.pose.pose.orientation.x
                yq = detection.pose.pose.pose.orientation.y
                zq = detection.pose.pose.pose.orientation.z
                wq = detection.pose.pose.pose.orientation.w
                [r,p,ya] = self.qte(wq,xq,yq,zq)
                roll = ya
                pitch = r+np.pi
                yaw = -p+np.pi
                cam_loc=np.array([x,y,z,roll,pitch,yaw])
                #rospy.loginfo("[%s] %s %s %s %s %s %s" %(self.node_name,x,y,z,roll*180/3.1415,pitch*180/3.1415,yaw*180/3.1415))
                tmp_6 = self.visual_odometry(cam_loc,detection.id[0])
                tmp=tmp+([tmp_6[0],tmp_6[1],tmp_6[2],math.sin(tmp_6[3]),math.cos(tmp_6[3]),math.sin(tmp_6[4]),math.cos(tmp_6[4]),math.sin(tmp_6[5]),math.cos(tmp_6[5])])
            if count != 0:
                roll = math.atan2(tmp[3],tmp[4])
                pitch = math.atan2(tmp[5],tmp[6])
                yaw = math.atan2(tmp[7],tmp[8])
                tmp = tmp/count
                secs = self.toSeconds(msg.header.stamp.secs,msg.header.stamp.nsecs)
                self.camera_motion = np.append(self.camera_motion,([[secs,tmp[0],tmp[1],tmp[2],roll,pitch,yaw]]),axis=0)
                #rospy.loginfo("[%s] %s %s %s %s %s %s" %(self.node_name,tmp[0],tmp[1],tmp[2],roll*180/3.1415,pitch*180/3.1415,yaw*180/3.1415))

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

    #switch for activating the node
    def cbSwitch(self, msg):
        self.active=msg.data

    #Flag for data gathering
    def cbRecord(self, msg):
        if msg.data:
            self.wheel_motion = np.zeros((1,3))
            self.camera_motion = np.zeros((1,7))
            self.data_gathering=msg.data

#########################################################################################################################################################################################
#Functions used in the calibration of the duckiebot
#########################################################################################################################################################################################

    #Determines the travel of the Duckiebot using the inputs sent to the motors
    def wheels(self,x,args):
        size=np.size(args,0)
        #get velocity and yaw rate from kinematic model
        vel=0.5*(np.dot(x[0],args[:,1])+np.dot(x[1],args[:,2]))*self.K
        #print vel
        #TODO changed the baseline to 1 single entity x[2], x[3]  should now be ignored in the algo
        omega=-np.dot(x[0]/x[2],args[:,1])*self.K+np.dot(x[1]/x[2],args[:,2])*self.K

        # print x
        # print args
        # print omega
        #extract initial yaw angle form yaw measurement of camera

        #yaw=np.zeros(size)
        yaw=np.zeros((size-1)*self.intersampling+1)

        yaw[0]=args[0][8]%(2*np.pi)

        # #calculate yaw angle for every single timestamp, using euler forward
        # for i in range(1,size):
        #     yaw[i]=yaw[i-1]+omega[i-1]*(args[i][0]-args[i-1][0])
        #calculate yaw angle for every single timestamp, using euler forward
        for i in range(1,(size-1)*self.intersampling+1):
            yaw[i]=yaw[i-1]+(omega[int(np.ceil(i/self.intersampling_f))-1]*(args[int(np.ceil(i/self.intersampling_f))][0]-args[int(np.ceil(i/self.intersampling_f))-1][0]))/self.intersampling_f
        x_est=0
        y_est=0
        #calculate x and y movement, using euler forward
        # for i in range (0,size-1):
        #     x_est=x_est+(args[i+1][0]-args[i][0])*math.cos(yaw[i])*vel[i]
        #     y_est=y_est+(args[i+1][0]-args[i][0])*math.sin(yaw[i])*vel[i]
        # return [x_est,y_est,0,0,0,(yaw[size-1]-yaw[0])%(2*np.pi)]

        #calculate x and y movement, using euler forward
        for i in range (1,(size-1)*self.intersampling+1):
            x_est=x_est+(args[int(np.ceil(i/self.intersampling_f))][0]-args[int(np.ceil(i/self.intersampling_f))-1][0])/self.intersampling_f*math.cos(yaw[i-1])*vel[int(np.floor(i/self.intersampling_f))]
            y_est=y_est+(args[int(np.ceil(i/self.intersampling_f))][0]-args[int(np.ceil(i/self.intersampling_f))-1][0])/self.intersampling_f*math.sin(yaw[i-1])*vel[int(np.floor(i/self.intersampling_f))]
        return [x_est,y_est,0,0,0,(yaw[(size-1)*self.intersampling]-yaw[0])%(2*np.pi)]

    #Determines the travel of the Duckiebot camera from Apriltag detections
    def visual_odometry(self,cam_loc,id):
        #Read tag location from yaml file
        tag_loc=self.tags_locations['tag'+str(id)]

        #This part can be done outside of the optimization algorithm --> save computation time
        #If more than 1 tag detected, take the average location of the N detections
        #Homogenuous transform World to Tag
        tag_T_world = self.etH(tag_loc)

        #Homogenuous transform Tag to Cam
        tag_T_cam = self.etH(cam_loc)
        cam_T_tag = np.linalg.inv(tag_T_cam)

        cam_T_world = np.dot(tag_T_world,cam_T_tag)

        R = cam_T_world[:3,:3]
        [roll,pitch,yaw] = self.Rte(R)
        return np.array([cam_T_world[0][3],cam_T_world[1][3],cam_T_world[2][3],roll,pitch,yaw])

    #Determines the travel of the Duckiebot from the movement of its camera
    def camera(self,x,cam_loc):
        #Homogenuous transform from camera to Duckiebot 'center'
        cam_T_bot = self.etH(x[4:])
        bot_T_cam = np.linalg.inv(cam_T_bot)

        #Homogenuous transform World to camera
        cam_T_world = self.etH(cam_loc)

        bot_T_world = np.dot(cam_T_world,bot_T_cam)
        R = bot_T_world[:3,:3]
        [roll,pitch,yaw] = self.Rte(R)
        yaw = yaw%(2*np.pi)
        return np.array([bot_T_world[0][3],bot_T_world[1][3],bot_T_world[2][3],roll,pitch,yaw])

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

            # args[][0] = timestamp
            # args[][1] = wl wheel velocity left
            # args[][2] = wr wheel velocity right
            # args[][3] = X camera position in X
            # args[][4] = Y camera position in Y
            # args[][5] = Z camera position in Z
            # args[][6] = camera roll
            # args[][7] = camera pitch
            # args[][8] = camera yaw

            n = 10
            x0 = np.zeros(n)
            # x0[0] = 0.033
            # x0[1] = 0.033
            # x0[2] = 0.1
            # x0[3] = 0.085
            # x0[4] = 0.055
            # x0[5] = 0
            # x0[6] = 0.1
            # x0[7] = 0
            # x0[8] = 0.349
            # x0[9] = 0

            x0[0] = 0.03
            x0[1] = 0.03
            x0[2] = 0.11
            x0[3] = 0.085
            x0[4] = 0.06
            x0[5] = 0.005
            x0[6] = 0.104
            x0[7] = 0.1
            x0[8] = 0.25
            x0[9] = 0.05

            # bounds
            b0 = (0.025,0.035)
            b1 = (0.025,0.035)
            b2 = (0.085,0.115)
            b3 = (0.085,0.115)
            b4 = (0.05,0.07)
            b5 = (-0.01,0.01)
            b6 = (0.09,0.11)
            b7 = (-0.174,0.174)
            b8 = (0.174,0.523)
            b9 = (-0.087,0.087)

            bnds = (b0, b1, b2, b3, b4, b5, b6, b7, b8, b9)

            #Array of camera positions and wheel velocities with corresponding timestamp

            # optimization
            solution = minimize(self.objective,x0, method='SLSQP',bounds=bnds)
            x = solution.x
            print x
            #self.finishCalc()

    #Objective function of the optimization algorithm
    def objective(self,x):
        camera_frames = np.size(self.camera_motion,0)
        wheel_frames = np.size(self.para,0)
        camera_pos = np.zeros((camera_frames,6))
        camera_mov = np.zeros((camera_frames-1,6))
        wheel_mov = np.zeros((camera_frames-1,6))

        #estimation by wheels
        start = -1
        stop = -1
        for i in range(0,camera_frames-1):
            #To prevent resampling errors (if motor and camera frame have exact same timeframe)
            start = -1
            stop = -1
            for j in range(0,wheel_frames):
                if (self.para[j][0]==self.camera_motion[i][0])&(start==-1):
                    start=j
                if (self.para[j][0]==self.camera_motion[i+1][0])&(stop==-1):
                    stop=j
            wheel_mov[i]=self.wheels(x,self.para[start:stop+1])

        #estimation by camera
        for i in range(0,camera_frames):
            camera_pos[i]=self.camera(x,self.camera_motion[i,1:])
            if i>0:
                camera_mov[i-1]=camera_pos[i]-camera_pos[i-1]
                camera_mov[i-1][5]=camera_mov[i-1][5]%(2*np.pi)

        #Create vectors to calculate the norm
        # print x
        # print wheel_mov
        # print camera_mov

        camera_mov=np.reshape(camera_mov,np.size(camera_mov))
        wheel_mov=np.reshape(wheel_mov,np.size(wheel_mov))
        #print x
        #print np.linalg.norm(wheel_mov-camera_mov)
        length = np.size(camera_mov)
        diff = np.zeros(length)
        diff = wheel_mov-camera_mov
        for i in range(0,length):
            if (i%6==5):
                if diff[i]>np.pi:
                    diff[i]=diff[i]-(2*np.pi)
                if diff[i]<-np.pi:
                    diff[i]=diff[i]+(2*np.pi)
        print np.linalg.norm(diff)
        return np.linalg.norm(diff)

    #secs and nsecs to secs double
    def toSeconds(self,secs,nsecs):
        return secs + nsecs/1000000000.0

    def resample(self):
        #delete first row of both arrays, as they are filled with zeros
        self.wheel_motion = np.delete(self.wheel_motion,0,0)
        self.camera_motion = np.delete(self.camera_motion,0,0)
        wheel_entries = np.size(self.wheel_motion,0)
        camera_entries = np.size(self.camera_motion,0)
        #delete last camera entry, to be sure that there are wheel entries coming after it
        self.camera_motion = np.delete(self.camera_motion,camera_entries-1,0)
        camera_entries=camera_entries-1
        for i in range(0,camera_entries):
            for j in range(0,wheel_entries):
                if (self.wheel_motion[j][0]>=self.camera_motion[i][0]):
                    self.wheel_motion = np.insert(self.wheel_motion,j,[self.camera_motion[i][0],self.wheel_motion[j-1][1],self.wheel_motion[j-1][2]],axis=0)
                    wheel_entries=wheel_entries+1
                    #delete all wheel entries before the first camera entry
                    if (i==0):
                        self.wheel_motion = np.delete(self.wheel_motion,slice(0,j),0)
                        wheel_entries = np.size(self.wheel_motion,0)
                    #delete all wheel entries after the last camera entry
                    if (i==camera_entries-1):
                        self.wheel_motion = np.delete(self.wheel_motion,slice(j+1,wheel_entries),0)
                    break

        #parameter vector to store all the robots motion (resampeld)
        self.para = self.wheel_motion

        self.para = np.append(self.para, np.zeros((np.size(self.para,0),6)),axis=1)
        wheel_entries = np.size(self.para,0)
        j = 0
        for i in range(0,wheel_entries):
            if (self.camera_motion[j][0]==self.para[i][0]):
                self.para[i][3] = self.camera_motion[j][1]
                self.para[i][4] = self.camera_motion[j][2]
                self.para[i][5] = self.camera_motion[j][3]
                self.para[i][6] = self.camera_motion[j][4]
                self.para[i][7] = self.camera_motion[j][5]
                self.para[i][8] = self.camera_motion[j][6]
                j=j+1

    #Taken from https://www.learnopencv.com/rotation-matrix-to-euler-angles/ on 19.6.18
    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    #Taken from https://www.learnopencv.com/rotation-matrix-to-euler-angles/ on 19.6.18
    # Calculates rotation matrix to euler angles
    def Rte(self,R) :

        assert(self.isRotationMatrix(R))
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return x, y, z

    #euler and translation to Homography
    #para is x,y,z,roll,pitch,yaw
    def etH(self,para):
        Rx = np.array([[1,0,0],
                       [0,np.cos(para[3]),-np.sin(para[3])],
                       [0,np.sin(para[3]), np.cos(para[3])]])
        Ry = np.array([[ np.cos(para[4]),0,np.sin(para[4])],
                       [0,1,0],
                       [-np.sin(para[4]),0,np.cos(para[4])]])
        Rz = np.array([[np.cos(para[5]),-np.sin(para[5]),0],
                       [np.sin(para[5]), np.cos(para[5]),0],
                       [0,0,1]])
        R = np.dot(np.dot(Rz,Ry),Rx)
        t = np.array([[para[0]],[para[1]],[para[2]]])
        return np.concatenate((np.concatenate((R,t), axis=1),np.array([[0,0,0,1]])), axis=0)

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
