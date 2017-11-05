#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose, BoolStamped, Twist2DStamped
from scipy.stats import multivariate_normal
from scipy.ndimage.filters import gaussian_filter
from math import floor, pi, sqrt

class PFLaneFilterNode(object):
    """

        PF Lane Filter Node

        Author: Miguel de la Iglesia Valls

        Inputs: SegmentList from line detector

        Outputs: LanePose - the d (lateral displacement) and phi (relative angle)
        of the car in the lane


    """
    def __init__(self):
        print 'miguel'
        self.node_name = "PF Lane Filter"
        self.active = True
        self.updateParams(None)

        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.beliefRV=np.empty(self.d.shape)

        self.initializeBelief()
        self.lanePose = LanePose()
        self.lanePose.d=self.mean_0[0]
        self.lanePose.phi=self.mean_0[1]

        self.particles = np.array([np.zeros(10), np.zeros(10)])
        # initialize with proper mean and covariance for the intitial guess.


        self.t_last_update = rospy.get_time()
        self.v_current = 0
        self.w_current = 0
        self.v_last = 0
        self.w_last = 0
        self.v_avg = 0
        self.w_avg = 0

        # Subscribers
        if self.use_propagation:
            self.sub_velocity = rospy.Subscriber("~velocity", Twist2DStamped, self.updateVelocity)
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)

        # Publishers
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)
        #self.pub_prop_img = rospy.Publisher("~prop_img", Image, queue_size=1)
        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def updateParams(self, event):
        self.mean_0 = [rospy.get_param("~mean_d_0",0) , rospy.get_param("~mean_phi_0",0)]
        self.cov_0  = [ [rospy.get_param("~sigma_d_0",0.1) , 0] , [0, rospy.get_param("~sigma_phi_0",0.01)] ]
        self.delta_d     = rospy.get_param("~delta_d",0.02) # in meters
        self.delta_phi   = rospy.get_param("~delta_phi",0.02) # in radians
        self.d_max       = rospy.get_param("~d_max",0.5)
        self.d_min       = rospy.get_param("~d_min",-0.7)
        self.phi_min     = rospy.get_param("~phi_min",-pi/2)
        self.phi_max     = rospy.get_param("~phi_max",pi/2)

        self.cov_v       = rospy.get_param("~cov_v",0.5) # linear velocity "input"
        self.cov_omega   = rospy.get_param("~cov_omega",0.01) # angular velocity "input"
        self.linewidth_white = rospy.get_param("~linewidth_white",0.04)
        self.linewidth_yellow = rospy.get_param("~linewidth_yellow",0.02)
        self.lanewidth        = rospy.get_param("~lanewidth",0.4)
        self.min_max = rospy.get_param("~min_max", 0.3) # nats

        # For use of maximum segment distance
        self.use_max_segment_dist = rospy.get_param("~use_max_segment_dist",False)
        self.max_segment_dist = rospy.get_param("~max_segment_dist",1.0)

        # For use of minimum segment count
        self.use_min_segs = rospy.get_param("~use_min_segs",False)
        self.min_segs = rospy.get_param("~min_segs", 10)

        # For propagation
        self.use_propagation = rospy.get_param("~use_propagation",False)
        self.cov_mask = [rospy.get_param("~sigma_d_mask",0.05) , rospy.get_param("~sigma_phi_mask",0.05)]

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def processSegments(self,segment_list_msg):
        if not self.active:
            return
#         t_start = rospy.get_time()

        if self.use_propagation:
            self.propagateBelief()

        for segment in segment_list_msg.segments:
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            d_i,phi_i,l_i = self.generateVote(segment)
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue
            if self.use_max_segment_dist and (l_i > self.max_segment_dist):
                continue
            measurement_dist=multivariate_normal(np.array([d_i, phi_i]), np.array([[1, 0],[0,1]]))
            self.weights *= measurement_dist.pdf(self.particles)

                #i = floor((d_i - self.d_min)/self.delta_d)
                #j = floor((phi_i - self.phi_min)/self.delta_phi)
                #measurement_likelihood[i,j] = measurement_likelihood[i,j] +  1


        self.weights = self.weights/np.sum(self.weights)

        resample_now = True
        if resample_now:
            self.resample()


        # TODO entropy test:
        #print self.beliefRV.argmax()

        maxids = np.unravel_index(self.beliefRV.argmax(),self.beliefRV.shape)
        # rospy.loginfo('maxids: %s' % maxids)
        self.lanePose.header.stamp = segment_list_msg.header.stamp
        self.lanePose.d = np.average(self.particles[:,0], self.weights)
        self.lanePose.phi = np.mean(self.particles[:,1], self.weights)
        self.lanePose.status = self.lanePose.NORMAL

        # publish the belief image
        bridge = CvBridge()
        #TODO count particles to form the beilive image
        belief_img = bridge.cv2_to_imgmsg((255*self.beliefRV).astype('uint8'), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp

        max_val = self.beliefRV.max()
        self.lanePose.in_lane = True #TODO: do proper test, like are 2 sigma particles in bounds?
        self.pub_lane_pose.publish(self.lanePose)
        self.pub_belief_img.publish(belief_img)

        # print "time to process segments:"
        # print rospy.get_time() - t_start

        # Publish in_lane according to the ent
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = self.lanePose.in_lane
        # ent = entropy(self.beliefRV)
        # if (ent < self.max_entropy):
        #     in_lane_msg.data = True
        # else:
        #     in_lane_msg.data = False
        self.pub_in_lane.publish(in_lane_msg)

    def updateVelocity(self,twist_msg):
        delta_t = rospy.get_time() - self.t_last_update

        self.d_phi += delta_t*twist_msg.omega
        self.d_d += delta_t*twist_msg.v*np.sin(self.d_phi)

        self.t_last_update = rospy.get_time()

        #self.v_avg = (self.v_current + self.v_last)/2.0
        #self.w_avg = (self.w_current + self.w_last)/2.0

        #self.v_last = v_current
        #self.w_last = w_current

    def initializeBelief(self): #TODO not used at the moment
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        self.beliefRV=RV.pdf(pos)

    def propagateBelief(self):

        self.particles[:,1] += self.d_phi + noise
        self.particles[:,0] += self.d_d*np.sin(self.particles[:,1]) + noise

        self.d_d = 0
        self.d_phi = 0

    def generateVote(self,segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2-p1)/np.linalg.norm(p2-p1)
        n_hat = np.array([-t_hat[1],t_hat[0]])
        d1 = np.inner(n_hat,p1)
        d2 = np.inner(n_hat,p2)
        l1 = np.inner(t_hat,p1)
        l2 = np.inner(t_hat,p2)
        if (l1 < 0):
            l1 = -l1;
        if (l2 < 0):
            l2 = -l2;
        l_i = (l1+l2)/2
        d_i = (d1+d2)/2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE: # right lane is white
            if(p1[0] > p2[0]): # right edge of white lane
                d_i = d_i - self.linewidth_white
            else: # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth/2

        elif segment.color == segment.YELLOW: # left lane is yellow
            if (p2[0] > p1[0]): # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else: # right edge of white lane
                d_i = -d_i
            d_i =  self.lanewidth/2 - d_i

        return d_i, phi_i, l_i

    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x)/2
        y_c = (segment.points[0].y + segment.points[1].y)/2

        return sqrt(x_c**2 + y_c**2)

    def onShutdown(self):
        rospy.loginfo("[PFLaneFilterNode] Shutdown.")


if __name__ == '__main__':
    rospy.init_node('pf_lane_filter',anonymous=False)
    pf_lane_filter_node = PFLaneFilterNode()
    rospy.on_shutdown(pf_lane_filter_node.onShutdown)
    rospy.spin()
