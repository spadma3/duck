#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from duckietown_msgs.msg import LanePose, BoolStamped, Twist2DStamped
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils import rgb_from_ros


class LaneFilterNode(object):
    def __init__(self):
        self.node_name = "Lane Filter"
        self.robot_name = rospy.get_param("~veh", "")
        self.active = True
        self.filter = None
        self.skip = 0
        self.updateParams(None)
        
        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()
        
        # Subscribers
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_velocity = rospy.Subscriber("/{}/car_cmd_switch_node/cmd".format(self.robot_name), Twist2DStamped, self.updateVelocity)
        sub_topic = '/{}/camera_node/image/compressed'.format(self.robot_name)
        self.subscriber = rospy.Subscriber(sub_topic, CompressedImage, self.processImage, queue_size=1)

        # Publishers
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
        self.pub_torch_im   = rospy.Publisher("~torch_im",Image, queue_size=1)

        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def updateParams(self, event):
        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c
            c[1]['robot_name'] = self.robot_name

            self.loginfo('new filter config: %s' % str(c))
            self.filter = instantiate(c[0], c[1])
            

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processImage(self, image):
        if not self.active:
            return
        self.skip = (self.skip + 1) % 10
        if self.skip != 0:
            return
        # Step 1: predict
        current_time = rospy.get_time()
        self.filter.predict(dt=current_time-self.t_last_update, v = self.velocity.v, w = self.velocity.omega)
        self.t_last_update = current_time

        # Step 2: update
        self.filter.update(rgb_from_ros(image))
        #bridge = CvBridge()
        #img_msg = bridge.cv2_to_imgmsg(img.astype('uint8'), 'bgr8')
        #self.pub_torch_im.publish(img_msg)

        # Step 3: build messages and publish things
        [d_max,phi_max] = self.filter.getEstimate()
        in_lane = self.filter.inLane()
        
        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = image.header.stamp
        lanePose.d = d_max
        lanePose.phi = phi_max
        lanePose.in_lane = in_lane
        lanePose.status = lanePose.NORMAL

        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg((255*self.filter.belief).astype('uint8'), "mono8")
        belief_img.header.stamp = image.header.stamp
        
        self.pub_lane_pose.publish(lanePose)
        self.pub_belief_img.publish(belief_img)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = image.header.stamp
        in_lane_msg.data = in_lane
        self.pub_in_lane.publish(in_lane_msg)

    def updateVelocity(self,twist_msg):
        self.velocity = twist_msg

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")


    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
