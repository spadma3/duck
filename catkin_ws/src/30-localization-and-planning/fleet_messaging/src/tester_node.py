#!/usr/bin/env python
import array
import rospy
from std_msgs.msg import ByteMultiArray

# Define callback function
def callback(msg):
    s = "Message received: %s %s %s" %(msg.data[0], msg.data[1], msg.data[2])
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
        rospy.loginfo("Message published")
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
        count = count + 1
        rospy.sleep(1.0)

if __name__ == '__main__':
    print "Running"
    listener()