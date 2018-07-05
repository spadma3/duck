#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int8, String, Bool
import time


# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7, logitek = 8, left joy = 9, right joy = 10
class LEDJoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        self.pub_pattern= rospy.Publisher("~change_color_pattern", String, queue_size=1)

        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_start = rospy.Subscriber("~start", Bool, self.cbStart, queue_size=1)
        self.startTime = 0
        self.last = 0

        self.button2patterns = {
             # 'a' is pressed
            0: 'CAR_SIGNAL_A',
            # 'b' is pressed
            1: 'tl_off',
	    #1: 'ON_RED',
            # 'Y' is pressed
            3: 'SIGNAL_GREEN'
            # 'X' is pressed
#            4: 'ON_BLUE',
            # lb is pressed
#            5: 'OFF',
            # rb is pressed
#             5: 'traffic_light_stop',
            # logitek button is pressed
#             8: 'test_all_1',
        }

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.startTime = time.time()
        #self.publishControl()
    def cbStart(self, flag):
        self.startTime = time.time()
        self.publishControl()



    # def publishControl(self):
    #
    #     for b, pattern in self.button2patterns.items():
    #         if self.joy.buttons[b] == 1:
    #             self.pub_pattern.publish(pattern)
    #             rospy.loginfo("Publishing pattern %s" % (pattern))

    def publishControl(self):
        dt = 5
        for i in range(0,2):
            self.pub_pattern.publish('CAR_SIGNAL_A')
            rospy.sleep(dt/pow(2,i))
            self.pub_pattern.publish('light_off')
            rospy.sleep(dt/pow(2,i))
        rospy.loginfo("###############Finished#############")

if __name__ == "__main__":
    rospy.init_node("led_joy_mapper",anonymous=False)
    led_joy_mapper = LEDJoyMapper()
    rospy.spin()
