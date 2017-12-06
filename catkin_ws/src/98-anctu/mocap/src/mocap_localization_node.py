#!/usr/bin/env python
import rospy
import numpy as np
import scipy as sp
from duckietown_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point


class MocapLocalizationNode(object):
    def __init__(self):
        self.node_name = "Mocap Localization" 
        # base tag id      
        self.tag_base_id = [129, 131, 39]
        # vehicle tag id    
        self.tag_vehicle_id = 8
        # base tag groundtruth point
        self.tag_base_point = np.array([[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0]], dtype='f')
        # base tag detection point     
        self.tag_oberv_point = np.zeros((3, 3), dtype='f')
        self.tag_test_point = np.zeros((3, 4), dtype='f')
        # vehicle tag detection point
        self.tag_vehicle_point = np.zeros((1, 4), dtype='f')
        # Subscribers
        self.sub_tag_detections = rospy.Subscriber("~tag_detections", AprilTagDetectionArray, self.processTagDetections, queue_size=1)
        # Publishers
        self.pub_vehicle_point = rospy.Publisher("~vehicle_point", Point, queue_size=1)
    def processTagDetections(self,tag_detections_msg):
        print "-----------------------------------------------"
        self.tag_base_point = np.array([[0, 0, 0], [1.5, 0, 0], [0, 1.5, 0]], dtype='f')
        for tag_detection in tag_detections_msg.detections:
            for index, tag_id in enumerate(self.tag_base_id):
                if tag_detection.id == tag_id:
                    self.tag_oberv_point[index, 0] = tag_detection.pose.pose.position.x
                    self.tag_oberv_point[index, 1] = tag_detection.pose.pose.position.y                   
                    self.tag_oberv_point[index, 2] = tag_detection.pose.pose.position.z
                    self.tag_test_point[index, 0] = tag_detection.pose.pose.position.x
                    self.tag_test_point[index, 1] = tag_detection.pose.pose.position.y                   
                    self.tag_test_point[index, 2] = tag_detection.pose.pose.position.z
                    self.tag_test_point[index, 3] = 1
            if tag_detection.id == self.tag_vehicle_id:
                    self.tag_vehicle_point[0, 0] = tag_detection.pose.pose.position.x
                    self.tag_vehicle_point[0, 1] = tag_detection.pose.pose.position.y                   
                    self.tag_vehicle_point[0, 2] = tag_detection.pose.pose.position.z
                    self.tag_vehicle_point[0, 3] = 1

        p_ct = (self.tag_base_point[0] + self.tag_base_point[1] + self.tag_base_point[2])/3
        p_cm = (self.tag_oberv_point[0] + self.tag_oberv_point[1] + self.tag_oberv_point[2])/3

        for i in range(3):
            self.tag_oberv_point[i] = self.tag_oberv_point[i] - p_cm
            self.tag_base_point[i] = self.tag_base_point[i] - p_ct

        Mtd = np.vstack((self.tag_base_point[0], self.tag_base_point[1], self.tag_base_point[2])).transpose()
        Mmd = np.vstack((self.tag_oberv_point[0], self.tag_oberv_point[1], self.tag_oberv_point[2])).transpose()

        H = np.dot(Mmd, Mtd.transpose())
        [U, D, V] = np.linalg.svd(H,full_matrices=1)
        R = np.dot(V,U.transpose())
        t = np.matrix(p_ct - np.dot(R,p_cm))

        temp = np.hstack((R,t.transpose()))
        zero = np.array([[0,0,0,1]])
        T = np.vstack((temp,zero))

        print "tag detection"
        print self.tag_test_point.transpose()
        print "tag detection after transformation"
        print np.dot(T,self.tag_test_point.transpose())

        print "vehicle tag detection"
        print self.tag_vehicle_point.transpose()
        print "vehicle tag detection after transformation"
        print np.dot(T,self.tag_vehicle_point.transpose())

        vehicle_point = np.zeros((1, 4), dtype='f')
        vehicle_point = np.dot(T,self.tag_vehicle_point.transpose())
        vehicle_point_msg = Point()
        vehicle_point_msg.x = vehicle_point[0]
        vehicle_point_msg.y = vehicle_point[1]
        vehicle_point_msg.z = vehicle_point[2]
        self.pub_vehicle_point.publish(vehicle_point_msg)

    def onShutdown(self):
        rospy.loginfo("[MocapLocalizationNode] Shutdown.")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('mocap_localization_node',anonymous=False)
    mocap_localization_node = MocapLocalizationNode()
    rospy.on_shutdown(mocap_localization_node.onShutdown)
    rospy.spin()
