#!/usr/bin/env python
import rospy, unittest, rostest
from duckietown_msgs.msg import LanePose, StopLineReading
#from duckietown_msgs.msg import messages to command the wheels
from duckietown_msgs.msg import Twist2DStamped
import time

class KinematicsTestNode(unittest.TestCase):
    def __init__(self, *args):
        super(KinematicsTestNode, self).__init__(*args)

    def setup(self):

        rospy.init_node('kinematics_test_node', anonymous=False)


        veh_name= rospy.get_param("~veh","")
        wheel_topic = "/" + veh_name + "/joy_mapper_node/car_cmd"


        rospy.loginfo("wheel topic = %s", wheel_topic)





        self.forward_time = 99

        self.pub_wheels_cmd = rospy.Publisher(wheel_topic,Twist2DStamped, queue_size=1)





    def test_drive_forward(self):
        self.setup()
        #move forward



        #Measured dist for stop as 146+8cm cm physically

        forward_for_time = self.forward_time
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time)):
            wheels_cmd_msg = Twist2DStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.v = 0.5
            wheels_cmd_msg.omega = 0.0
            self.pub_wheels_cmd.publish(wheels_cmd_msg)
            #rospy.loginfo("Moving?.")
            rospy.sleep(0.1)








if __name__ == '__main__':
    rostest.rosrun('rostest_kinematics_calibration', 'kinematics_test_node', KinematicsTestNode)
