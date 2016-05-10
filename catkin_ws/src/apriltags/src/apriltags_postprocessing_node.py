#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTags, TagDetection, TagInfo, Vector2D, VehiclePose
import numpy as np
import kinematic as k

class AprilPostPros(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltags')
        # No tags_file input atm., so default value is used
        tags_filepath = self.setupParam("~tags_file", self.pkg_path+"/apriltagsDB/apriltagsDB.yaml") 
        self.loc = self.setupParam("~loc", -1) # -1 if no location is given
        tags_file = open(tags_filepath, 'r')
        self.tags_dict = yaml.load(tags_file)
        tags_file.close()
        self.info = TagInfo()
 
        self.sign_types = {"StreetName": self.info.S_NAME,
            "TrafficSign": self.info.SIGN,
            "Light": self.info.LIGHT,
            "Localization": self.info.LOCALIZE,
            "Vehicle": self.info.VEHICLE}
        self.traffic_sign_types = {"stop": self.info.STOP, 
            "yield": self.info.YIELD, 
            "no-right-turn": self.info.NO_RIGHT_TURN, 
            "no-left-turn": self.info.NO_LEFT_TURN, 
            "oneway-right": self.info.ONEWAY_RIGHT, 
            "oneway-left": self.info.ONEWAY_LEFT, 
            "4-way-intersect": self.info.FOUR_WAY, 
            "right-T-intersect": self.info.RIGHT_T_INTERSECT, 
            "left-T-intersect": self.info.LEFT_T_INTERSECT,
            "T-intersection": self.info.T_INTERSECTION,
            "do-not-enter": self.info.DO_NOT_ENTER,
            "pedestrian": self.info.PEDESTRIAN,
            "t-light-ahead": self.info.T_LIGHT_AHEAD,
            "duck-crossing": self.info.DUCK_CROSSING}

        self.sub_prePros        = rospy.Subscriber("~apriltags_in", AprilTags, self.callback, queue_size=1)
        self.pub_pose            = rospy.Publisher("~target_pose", VehiclePose, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, msg):
        """ """ 
        # Load tag detections message
        target_pose_out = VehiclePose()
        target_pose_out.detection = False
        if len(msg.detections) == 0:
          self.pub_pose.publish(target_pose_out)
          return
        target_pose_out.detection = True
        detection = msg.detections[0]  
        camera_x     = rospy.get_param("~camera_x")
        camera_y     = rospy.get_param("~camera_y")
        camera_z     = rospy.get_param("~camera_z")
        camera_theta = rospy.get_param("~camera_theta")
        scale        = rospy.get_param("~scale")
        #Load translation
        x = detection.transform.translation.x
        y = detection.transform.translation.y
        z = detection.transform.translation.z
        # translation tags(t) w/ camera(c) expressed in camera frame (Fc)
        t_tc_Fc = k.Vector( x , y , z ) 
        t_tc_Fc =  t_tc_Fc * scale
        #Load rotation
        x = detection.transform.rotation.x
        y = detection.transform.rotation.y 
        z = detection.transform.rotation.z
        w = detection.transform.rotation.w
        e = k.Vector( x , y , z )
        Q_Ftag_Fold = k.Quaternion( e , w )
        # New tag orientation reference (zero when facing camera) w/ to old tag ref used by the lib
        C_Ft_Ftag = k.RotationMatrix( np.matrix([[0,0,-1],[-1,0,0],[0,1,0]]) )
        Q_Ft_Ftag = C_Ft_Ftag.toQuaternion()
        # Rotation of old ref frame used by the lib w/ to camera frame
        C_Fold_Fc = k.RotationMatrix( np.matrix([[0,-1,0],[0,0,-1],[1,0,0]]) )
        Q_Fold_Fc = C_Fold_Fc.toQuaternion()
        # Camera localization
        # translation of camera w/ vehicle origin in vehicle frame
        t_cv_Fv = k.Vector( camera_x , camera_y , camera_z ) 
        C_Fc_Fv = k.euler2RotationMatrix(0,camera_theta,0)   # Rotation   of camera frame w/ vehicle frame
        Q_Fc_Fv = C_Fc_Fv.toQuaternion()
        # Compute tag orientation in vehicle frame
        Q_Ft_Fv =  Q_Fc_Fv * Q_Fold_Fc * Q_Ftag_Fold * Q_Ft_Ftag
        # Compute position of tag in vehicle frame expressed in vehicle frame
        C_Fv_Fc = - C_Fc_Fv # inverse transform
        t_tc_Fv = C_Fv_Fc * t_tc_Fc
        t_tv_Fv = t_tc_Fv + t_cv_Fv
        # Overwrite transformed value
        detection.transform.translation.x = t_tv_Fv.x
        detection.transform.translation.y = t_tv_Fv.y
        detection.transform.translation.z = t_tv_Fv.z
        detection.transform.rotation.x    = Q_Ft_Fv.e.x
        detection.transform.rotation.y    = Q_Ft_Fv.e.y
        detection.transform.rotation.z    = Q_Ft_Fv.e.z
        detection.transform.rotation.w    = Q_Ft_Fv.n
        A_Ft_Fv      = Q_Ft_Fv.toAngleAxis()
        # Publish Message
        target_pose_out.x = detection.transform.translation.x
        target_pose_out.y = detection.transform.translation.y
        target_pose_out.rho = np.linalg.norm([target_pose_out.x, target_pose_out.y])
        target_pose_out.theta = np.arctan2(target_pose_out.y, target_pose_out.x)
        target_pose_out.psi = 2 * detection.transform.rotation.z
        self.pub_pose.publish(target_pose_out)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
