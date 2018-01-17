#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import BoolStamped, Int16

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    pub = rospy.Publisher('bluck/intersection_navigation_node/switch', BoolStamped, queue_size=1)
	pub2 = rospy.Publisher('bluck/intersection_navigation_node/turn_type', Int16, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(BoolStamped(data=True))
		pub2.publish((Int16(1))
        rate.sleep()
