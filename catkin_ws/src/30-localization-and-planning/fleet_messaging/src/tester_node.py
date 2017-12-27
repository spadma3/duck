#!/usr/bin/env python
import array
import rospy
from std_msgs.msg import ByteMultiArray

# Define callback function
def callback(msg):
    s = "[tester_node] Message subscribed: %s %s %s" %(msg.data[0], msg.data[1], msg.data[2])
    rospy.loginfo(s)

def listener():
    #initialize node
    rospy.init_node('tester_node', anonymous=False)

    #define publisher and subscriber
    pub = rospy.Publisher("test_receive", ByteMultiArray, queue_size=1)
    sub = rospy.Subscriber("test_send", ByteMultiArray, callback)

    #initialize counter
    count = 0
    # mutable_bytes = bytearray(3)

    while not rospy.is_shutdown():
        msg = ByteMultiArray()
        # mutable_bytes[0] = count
        # mutable_bytes[1] = count + 1
        # mutable_bytes[2] = count + 2
        # msg.data.append(mutable_bytes[0])
        # msg.data.append(mutable_bytes[1])
        # msg.data.append(mutable_bytes[2])
        msg.data.append(count)
        msg.data.append(count + 1)
        msg.data.append(count + 2)
        pub.publish(msg)
        s = "[tester_node] Message published: %s %s %s" %(msg.data[0], msg.data[1], msg.data[2])
        rospy.loginfo(s)
        if count < 253:
            count = count + 1
        else:
            count = 0
        rospy.sleep(1.0)

if __name__ == '__main__':
    print "[tester_node] Running"
    listener()
