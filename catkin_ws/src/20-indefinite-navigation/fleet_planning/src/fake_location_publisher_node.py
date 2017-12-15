#!/usr/bin/env python
import rospy
import tf

from duckietown_msgs.msg import BoolStamped


class FakeLocationPublisher:
    def __init__(self):

        self.at_stop_line_publisher = rospy.Publisher(
            '~/jeff/stop_line_filter_node/at_stop_line', BoolStamped, queue_size=1, latch=True)
        self.location_publisher = tf.TransformBroadcaster()
        # publish a new location periodically
        self._publish_timer = rospy.Timer(
            rospy.Duration.from_sec(2.0), self.publish_fake_location)  # TODO: add argument 'duckiebot_name' to callback

        # self.location_x = 0.0

    def publish_fake_location(self, duckiebot_name = "/jeff1"):
        # t = tf.
        self.location_x = 2.0
        self.location_y = 1.0
        self.location_publisher.sendTransform((self.location_x, self.location_x, 0),
                                              tf.transformations.quaternion_from_euler(0, 0, 4),
                                              rospy.Time.now(),
                                              "duckiebot", #duckiebot_name,
                                              "world")
        status_msg = BoolStamped()
        status_msg.data = 1
        status_msg.header.stamp = rospy.Time.now()
        self.at_stop_line_publisher.publish()  # TODO: make sure this resembles the actual way this message is published

        # self.location_x += 1


if __name__ == '__main__':
    # startup node
    rospy.init_node('fake_location_publisher_node')
    fake_location_publisher = FakeLocationPublisher()
    
    rospy.spin()
