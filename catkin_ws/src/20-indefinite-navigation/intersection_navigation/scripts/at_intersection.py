#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    pub = rospy.Publisher('/intersection_navigation_node/mode', FSMState, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(FSMState(state="INTERSECTION_CONTROL"))
        rate.sleep()