#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import BoolStamped

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    pub = rospy.Publisher('bluck/intersection_navigation_node/switch', BoolStamped, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(BoolStamped(state=True))
        rate.sleep()
