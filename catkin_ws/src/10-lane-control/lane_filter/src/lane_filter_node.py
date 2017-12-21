#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from std_msgs.msg import Float32
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose, BoolStamped, Twist2DStamped
from duckietown_utils.instantiate_utils import instantiate
import sys
import os
import numpy as np
from matplotlib import pyplot as plt

class LaneFilterNode(object):
    def __init__(self):
        self.node_name = "Lane Filter"
        self.active = True
        self.filter = None
        self.updateParams(None)
        
        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()
        self.d_median = []
        self.phi_median = []
        self.store_d =[]
        self.store_phi =0
        
        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)

        # Publishers
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_ml_img = rospy.Publisher("~ml_img",Image,queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
      
        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def updateParams(self, event):
        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new filter config: %s' % str(c))
            self.filter = instantiate(c[0], c[1])
            

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processSegments(self,segment_list_msg):
        if not self.active:
            return

        # Step 1: predict
        current_time = rospy.get_time()
        self.filter.predict(dt=current_time-self.t_last_update, v = self.velocity.v, w = self.velocity.omega)
        self.t_last_update = current_time

        # Step 2: update
        range_arr = np.zeros(self.filter.num_belief+1)
        range_max = 0.6  # range to consider edges in general
        range_min = 0.2
        range_diff = (range_max - range_min)/(self.filter.num_belief - 1)
        
        for i in range(1,self.filter.num_belief + 1):
            range_arr[i] = range_min + (i-1)*range_diff

        self.filter.update(segment_list_msg.segments, range_arr)
        print "segment list", len(segment_list_msg.segments)

        # Step 3: build messages and publish things
        [d_max,phi_max, curvature] = self.filter.getEstimate()
        print curvature 
        print "phi max",phi_max
        print "d_max", d_max

        max_val = self.filter.getMax()
        in_lane = max_val > self.filter.min_max

        # check if the current estimation is an outlier compare to the last 3 estimates
        d= 0
        phi=0
        if len(self.store_d)<3:
            self.store_d.append(d_max[0])
            self.store_phi = phi_max[0]
            d=d_max[0]
            phi=phi_max[0]
        else:
            # if yes take the top one of the array give it to lane pose and store it again into the array. 
            if abs(np.mean(self.store_d[0:2])-d_max[0])> 0.06 :
                d =self.store_d[2]
                phi =self.store_phi
                self.store_d.pop(0)
                self.store_d.append(self.store_d[1])
                print "rejected"
            # else take the new d and store it i
            else:
                d = d_max[0]
                phi =phi_max[0]
                self.store_d.pop(0)
                self.store_d.append(d_max[0])
                self.store_phi = phi_max[0]
                print "accepted" , d,phi
            print "length stored d " , len(self.store_d), self.store_d
     




        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d =d
        lanePose.phi=phi
        lanePose.in_lane = in_lane
        lanePose.status = lanePose.NORMAL
        lanePose.curvature= curvature
    #anna 
        # if (me_phi_l<-0.2 and  av_d_l>0.03):
        #     print "I see a left curve"
        #     lanePose.curvature =0.025
        # elif (me_phi_l>0.2 and av_d_l<-0.03):
        #     print "I see a right curve"
        #     lanePose.curvature=0.054
        # else:
        #     print "I am on a straight line" 
        #     lanePose.curvature=0
        # print "curv ", lanePose.curvature
        # publish the belief image

    #simon
        #print "Delta dmax", delta_dmax
        #print "Delta phimax", delta_phimax
        # if np.median(self.phi_median) < -0.3 and np.median(self.d_median) > 0.05:
        #     print "left curve"
        #     lanePose.curvature = 0.025
        # elif np.median(self.phi_median) > 0.2 and np.median(self.d_median) < -0.02:
        #     print "right curve"
        #     lanePose.curvature = -0.054
        # else:
        #     print "straight line"
        #     lanePose.curvature = 0.0

        


        #  publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg((255*self.filter.beliefArray[0]).astype('uint8'), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp
        

        self.pub_lane_pose.publish(lanePose)
        self.pub_belief_img.publish(belief_img)

        #  also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = in_lane
        self.pub_in_lane.publish(in_lane_msg)

    def getDistributionImage(self,mat,stamp):
        bridge = CvBridge()
        img = bridge.cv2_to_imgmsg((255*mat).astype('uint8'), "mono8")
        img.header.stamp = stamp
        return img
        
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
