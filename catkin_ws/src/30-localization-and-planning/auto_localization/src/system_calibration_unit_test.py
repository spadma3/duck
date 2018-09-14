#!/usr/bin/env python


import rospkg
import rospy
from duckietown_msgs.msg import GlobalPoseArray, GlobalPose, RemapPose, RemapPoseArray
import global_pose_functions as gposf
import time
import numpy as np

class system_calibration_unit_test(object):
    def __init__(self):
        self.node_name = 'system_calibration_unit_test'
        self.pub_tfs = rospy.Publisher("/duckietown3/system_calibration_gui/complete_local_poses", RemapPoseArray, queue_size=1)
        time.sleep(3) #to be sure the system calibration node booted
        msg = self.generate_message()
        print msg
        self.pub_tfs.publish(msg)
        # time.sleep(10)
        rospy.signal_shutdown("Published Message")


    def generate_message(self):
        trans   = [1, -1, 0]
        rot     = [0, 1, 0, 0]
        host    = "watchtower01"
        msg = RemapPoseArray()
        # create a grid of apriltags
        for i in xrange(1,1000):
            for row in xrange(1,20):
                msg.poses.append(gposf.pose_msg_from_trans_rot(row,row+1, trans+0*np.random.rand(3,), rot+0.001*np.random.rand(4,), host))
                    # msg.poses.append(gposf.pose_msg_from_trans_rot(column,row, trans, rot, host))
                # for row in xrange(1,20):

            return msg



if __name__ == '__main__':
    rospy.init_node('system_calibration_unit_test',anonymous=False)
    node = system_calibration_unit_test()
    rospy.spin()
