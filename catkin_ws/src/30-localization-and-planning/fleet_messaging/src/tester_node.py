#!/usr/bin/env python
import rospy
from std_msgs.msg import ByteMultiArray

# Define callback function
def callback(msg):
    s = "Message recieved %s" % (msg.data)
    #rospy.loginfo(s)

def listener():
    #initialize node
    rospy.init_node('dummyTest_node', anonymous=False)

    #define publisher and subscriber
    pub = rospy.Publisher('test_send', ByteMultiArray, queue_size=1)
    sub = rospy.Subscriber("test_recieve", ByteMultiArray, callback)

    #define counting order
    count = 0

    while not rospy.is_shutdown():
        msg = ByteMultiArray()
        msg.data = count
        pub.publish(msg)
        count = count + 1
        # rospy.loginfo(msg.data)
        rospy.sleep(1.0)

if __name__ == '__main__':
    print "Running"
    listener()