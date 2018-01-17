#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import BoolStamped

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    pub = rospy.Publisher('daisy/intersection_navigation_node/in_lane', BoolStamped, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = BoolStamped
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        pub.publish(msg)
        rate.sleep()