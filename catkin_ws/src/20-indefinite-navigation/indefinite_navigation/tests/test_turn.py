#!/usr/bin/env python
import rospy, unittest, rostest
import math
from intersection_control.util import HelloGoodbye #Imports module. Not limited to modules in this pkg.
from duckietown_msgs.msg import LanePose, StopLineReading
from std_srvs.srv import Empty, EmptyRequest
from std_msgs.msg import String #Imports msg
from std_msgs.msg import Bool #Imports msg
#from duckietown_msgs.msg import messages to command the wheels
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, FSMState

class IndefNavigationTurnNode(unittest.TestCase):
    def __init__(self, *args):
        super(IndefNavigationTurnNode, self).__init__(*args)
        #self.setup()
    def setup(self):
        #give lane_filter some time to start before beginning
        rospy.sleep(8)
        rospy.init_node('indef_navigation_turn_node', anonymous=False)
        # Save the name of the node
        self.node_name = rospy.get_name()

        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        veh_name= self.setupParam("~veh", "")
        self.type = right
        wheels_cmd = "/" + veh_name + "/wheels_driver_node/car_cmd"

        self.publish_mode = rospy.Publisher(mode_topic, FSMState, queue_size=1)
        self.pub_wheels = rospy.Publisher(wheels_cmd, Twist2DStamped, queue_size=1)

        self.rate = rospy.Rate(30) # 10hz
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value




    def test_turn(self):
        self.setup()
        mode = FSMState()
        mode.state = "INTERSECTION_CONTROL"

        self.publish_mode.publish(mode)




        stop.v = 0.1
        stop.omega = 0



        self.pub_wheels.publish(stop)
            rospy.sleep(0.1)





if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('indef_navigation_turn_node', anonymous=False)

    rostest.rosrun('rostest_turn_calibration', 'indef_navigation_turn_node', IndefNavigationTurnNode)
