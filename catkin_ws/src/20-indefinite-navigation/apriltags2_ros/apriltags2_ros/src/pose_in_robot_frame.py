#!/usr/bin/env python
import rospkg
import rospy, math
import yaml
import numpy as np

from duckietown_msgs.msg import RemapPose, RemapPoseArray
from duckietown_utils import tcp_communication
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

import tf.transformations as tr
# Read clients (watchtower), read from yaml that describe the map in the future.

class AprilTagPostProcesser(object):

    def __init__(self):

        # Save the name of the node
        self.node_name = rospy.get_name()
# Start PARAMS 1
        self.mode = None
        self.triggered = False
        self.count_last = 0
        self.data_gathering = False #ONLY for testing purposes, default is FALSE
        # Number of steps to integrate omega for wheelspeeds
        self.intersampling = 1
        self.intersampling_f = 1.0
        #timestamp, x , y, z, roll, pitch, yaw
        self.camera_motion = np.zeros((1,7))
        #determined 9.35*math.pi by averaging Duckiebot speeds (in rad/s), this value taken from calibration files
        self.K = 27.0

        self.rate = rospy.Rate(30)
# End PARAMS 1

# Start PARAMS 2
        # Load parameters
        self.camera_x     = self.setupParam("~camera_x", 0.065)
        self.camera_y     = self.setupParam("~camera_y", 0.0)
        self.camera_z     = self.setupParam("~camera_z", 0.11)
        self.camera_theta = self.setupParam("~camera_theta", 19.0)
        self.scale_x     = self.setupParam("~scale_x", 1)
        self.scale_y     = self.setupParam("~scale_y", 1)
        self.scale_z = self.setupParam("~scale_z", 1)
# End PARAMS 2

        # Subscribers
        self.sub_tag_detection = rospy.Subscriber("/ata/tag_detections", AprilTagDetectionArray, self.callback, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        #rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    def cbTagDetection(self,tag_detection_msg):

        pose = RemapPose()
        detections = tag_detection_msg.detections[0]
        pose.bot_id = detections.id

        pose.posestamped.header.stamp.secs = detections.pose.header.stamp.secs
        pose.posestamped.header.stamp.nsecs = detections.pose.header.stamp.nsecs
        pose.posestamped.pose.position.x = detections.pose.pose.pose.position.x
        pose.posestamped.pose.position.y = detections.pose.pose.pose.position.y
        pose.posestamped.pose.position.z = detections.pose.pose.pose.position.z
        pose.posestamped.pose.orientation.x = detections.pose.pose.pose.orientation.x
        pose.posestamped.pose.orientation.y = detections.pose.pose.pose.orientation.y
        pose.posestamped.pose.orientation.z = detections.pose.pose.pose.orientation.z
        pose.posestamped.pose.orientation.w = detections.pose.pose.pose.orientation.w

        x = detections.pose.pose.pose.position.x
        y = detections.pose.pose.pose.position.y
        z = detections.pose.pose.pose.position.z
        xq = detections.pose.pose.pose.orientation.x
        yq = detections.pose.pose.pose.orientation.y
        zq = detections.pose.pose.pose.orientation.z
        wq = detections.pose.pose.pose.orientation.w

        #Coordinate transformation
        x = -detections.pose.pose.pose.position.z
        y = detections.pose.pose.pose.position.x
        z = detections.pose.pose.pose.position.y
        xq = detections.pose.pose.pose.orientation.x
        yq = detections.pose.pose.pose.orientation.y
        zq = detections.pose.pose.pose.orientation.z
        wq = detections.pose.pose.pose.orientation.w

        [r,p,ya] = self.qte(wq,xq,yq,zq)
        # SE
        roll = -ya
        pitch = r #+np.pi
        yaw = p #+np.pi

        """
        print "x: {} y: {} z: {} roll: {} pitch: {} yaw: {}".format(x,y,z,roll, pitch ,yaw)
        """
        # Ben Weber
        roll = ya
        pitch = r+np.pi
        yaw = -p+np.pi

        #print "x: {} y: {} z: {} roll: {} pitch: {} yaw: {}".format(x,y,z,roll, pitch,yaw)
        #cam_loc=np.array([x,y,z,roll,pitch,yaw])

        #tmp_6 = self.camera(cam_loc,detections.id[0])

        #tmp=tmp+([tmp_6[0],tmp_6[1],tmp_6[2],math.sin(tmp_6[3]),math.cos(tmp_6[3]),math.sin(tmp_6[4]),math.cos(tmp_6[4]),math.sin(tmp_6[5]),math.cos(tmp_6[5])])

        return pose

    def callback(self, tag_detection_msg):

        # Define the transforms
        veh_t_camxout = np.array((self.camera_x, self.camera_y, self.camera_z))
        veh_R_camxout = tr.euler_matrix(0, self.camera_theta*np.pi/180, 0, 'rxyz')
        veh_T_camxout = self.toHomegenousTransformation(veh_R_camxout, veh_t_camxout)  # 4x4 Homogeneous Transform Matrix


        camxout_T_camzout = tr.euler_matrix(-np.pi/2,0,-np.pi/2,'rxyz')
        veh_T_camzout = tr.concatenate_matrices(camxout_T_camzout, veh_T_camxout)

        #tagzout_T_tagxout = tr.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')

        #Load translation
        detection = tag_detection_msg.detections[0]

        trans = detection.pose.pose.pose.position
        rot = detection.pose.pose.pose.orientation
        header = detection.pose.header

        """
        x: -0.00369811188967
        y: 0.0011669218875
        z: 0.181002888573
      orientation:
        x: 0.97931277696
        y: -0.00501954642485
        z: 0.200129473306
        w: -0.029486996296

        """
        """
        #print "BEFORE TRANSFORM"
        print "x: {} y: {} z: {} rot x: {} rot y: {} rot z: {}".format(trans.x,trans.y,trans.z, rot.x , rot.y , rot.z)
        """

        tagout_t_camzout = np.array((trans.x*self.scale_x, trans.y*self.scale_y, trans.z*self.scale_z))
        tagout_R_camzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
        tagout_T_camzout = self.toHomegenousTransformation(tagout_R_camzout, tagout_t_camzout)
        #camzout_T_tagout = tr.inverse_matrix(tagout_T_camzout)
        camzout_T_tagout = tagout_T_camzout
        veh_T_tagout = tr.concatenate_matrices(camzout_T_tagout, veh_T_camzout)

        # Overwrite transformed value
        (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagout)
        (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagout)

        rotx = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[0] * 180 / np.pi
        roty = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[1] * 180 / np.pi
        rotz = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[2] * 180 / np.pi


        print "AFTER TRANSFORM"
        print "x: {} y: {} z: {} rot x: {} rot y: {} rot z: {}".format(trans.x,trans.y,trans.z, rotx , roty , rotz)


        """
        new_tag_data = RemapPose()
        new_tag_data.detections.append(AprilTagDetection(int(detection.id[0]),float(detection.size[0]),PoseStamped(header,Pose(trans,rot))))

        # Publish Message
        self.pub_postPros.publish(new_tag_data)
        """
    def toHomegenousTransformation(self,M,t):
        M = M[0:3,0:3]
        t = np.transpose([t.copy()])

        #print "M size: {}".format(M.shape)
        #print "t size: {}".format(t.shape)
        T_pre = np.concatenate((M, t), axis=1)
        T = np.concatenate((T_pre,np.array([[0,0,0,1]]) ), axis=0)

        return T

    #extract the camera motion from apriltags
    def cbTag(self, msg):
        if self.data_gathering:
            count = 0
            tmp = np.zeros(9)
            tmp_6 = np.zeros(6)
            for detection in msg.detections:
                if (detection.id[0]>=300 & detection.id[0]<=329):
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
                    tmp_6 = self.visual_odometry(cam_loc,detection.id[0])
                    tmp=tmp+([tmp_6[0],tmp_6[1],tmp_6[2],math.sin(tmp_6[3]),math.cos(tmp_6[3]),math.sin(tmp_6[4]),math.cos(tmp_6[4]),math.sin(tmp_6[5]),math.cos(tmp_6[5])])
                if count != 0:
                    roll = math.atan2(tmp[3],tmp[4])
                    pitch = math.atan2(tmp[5],tmp[6])
                    yaw = math.atan2(tmp[7],tmp[8])
                    tmp = tmp/count
                    secs = self.toSeconds(msg.header.stamp.secs,msg.header.stamp.nsecs)
                    self.camera_motion = np.append(self.camera_motion,([[secs,tmp[0],tmp[1],tmp[2],roll,pitch,yaw]]),axis=0)
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

if __name__ == '__main__':

    rospy.init_node('april_tag_post_processor',anonymous=False)

    node = AprilTagPostProcesser()

    # Setup proper shutdown behavior
    #rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
