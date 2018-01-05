import rospy
from duckietown_msgs.msg import WheelsCmdStamped
import math.pi
class LookAsideNode():

    def __init__(self):
        self.node_name=rospy.get_name()

        self.look_aside_pub = rospy.Publisher("~flag_looking_aside",
				BoolStamped, self.callback,  queue_size = 1)
        self.car_cmd_pub = rospy.Publisher("~car_cmd",
                                           Twist2DStamped, queue_size=1)

        self.at_intersection_sub = rospy.Subscriber("~flag_at_intersection", BoolStamped, self.cbStop)
        self.implicit_coordination_sub = rospy.Subscriber("~flag_intersection_wait_go_implicit", BoolStamped, self.cbImplicit, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd_in", Twist2DStamped, self.cbCarCmd, queue_size=1)
        self.v
        self.omega
        self.angle=math.pi/4
        self.duration=1




    def cbWheelSwitch(self,switch_msg):
        rospy.loginfo('SWITCH OUTPUT : (left = %.2f, right = %.2f)' %
            (switch_msg.vel_left, switch_msg.vel_right))

    def cbImplicit(self):
        car_cmd_msg_current = Twist2DStamped()
        car_cmd_msg_current.v=0
        start_time = rospy.Time.now()
        middle_time = start_time + rospy.Duration.from_sec(self.duration)
        end_time = start_time+2*rospy.Duration.from_sec(self.duration)
        while rospy.Time.now() < middle_time:
            car_cmd_msg_current.omega=-self.angle
            self.car_cmd_pub.publish(car_cmd_msg_current)
        while rospy.Time.now() < end_time:
            car_cmd_msg_current.omega = self.angle
            self.car_cmd_pub.publish(car_cmd_msg_current)





if __name__ == '__main__':
    rospy.init_node('look-aside', anonymous=False)
    look_asider=LookAsideNode()
    rospy.spin()
