from duckietown_utils.parameters import Configurable
from duckietown_msgs.msg import Segment
import numpy as np
from .lane_filter_interface import LaneFilterInterface
from scipy.stats import multivariate_normal
from scipy.ndimage.filters import gaussian_filter
from math import floor, pi, sqrt
import copy
import cv2
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError



class LaneFilterHistogram(Configurable, LaneFilterInterface):
    """LaneFilterHistogram"""

    def __init__(self,configuration):
        param_names = [
            'mean_d_0',
            'mean_phi_0',
            'sigma_d_0',
            'sigma_phi_0',
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            'cov_v',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'min_max',
            'sigma_d_mask',
            'sigma_phi_mask',
        ]
        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self,param_names,configuration)

        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.belief=np.empty(self.d.shape)
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0  = [ [self.sigma_d_0, 0], [0, self.sigma_phi_0] ]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        self.initialize()
        self.pub_img = rospy.Publisher("~some_img", Image, queue_size=1)

        self.phi_last = 0.0

    def predict(self, dt, v, w):
        delta_t = dt
        d_t = self.d + v*delta_t*np.sin(self.phi)
        phi_t = self.phi + w*delta_t

        p_belief = np.zeros(self.belief.shape)

        # there has got to be a better/cleaner way to do this - just applying the process model to translate each cell value
        for i in range(self.belief.shape[0]):
            for j in range(self.belief.shape[1]):
                if self.belief[i,j] > 0:
                    if d_t[i,j] > self.d_max or d_t[i,j] < self.d_min or phi_t[i,j] < self.phi_min or phi_t[i,j] > self.phi_max:
                        continue
                    i_new = int(floor((d_t[i,j] - self.d_min)/self.delta_d))
                    j_new = int(floor((phi_t[i,j] - self.phi_min)/self.delta_phi))
                    p_belief[i_new,j_new] += self.belief[i,j]

        s_belief = np.zeros(self.belief.shape)
        gaussian_filter(p_belief, self.cov_mask, output=s_belief, mode='constant')

        if np.sum(s_belief) == 0:
            return
        self.belief = s_belief/np.sum(s_belief)


    def update(self, segments):
        measurement_likelihood = self.generate_measurement_likelihood(segments)
        if measurement_likelihood is not None:
            self.belief = np.multiply(self.belief,measurement_likelihood)
            if np.sum(self.belief) == 0:
                self.belief = measurement_likelihood
            else:
                self.belief = self.belief/np.sum(self.belief)
        return measurement_likelihood

    def generate_measurement_likelihood(self, segments):
        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape)


        y_picmin = -0.4
        y_picmax = 0.4
        x_picmin = 0.0
        x_picmax = 1.4
        picres = 0.0075

        px_x = (x_picmax - x_picmin)/picres
        px_y = (y_picmax - y_picmin)/picres

        img = np.zeros((px_y,px_x,3), np.uint8)


        for segment in segments:
            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue



            if not (x_picmin < segment.points[0].x < x_picmax):
                continue
            if not (x_picmin < segment.points[1].x < x_picmax):
                continue
            if not (y_picmin < segment.points[0].y < y_picmax):
                continue
            if not (y_picmin < segment.points[1].y < y_picmax):
                continue

            picpoint1_x =  int(round(segment.points[0].x/(x_picmax - x_picmin) * px_x))
            picpoint1_y =  int(round(segment.points[0].y/(y_picmax - y_picmin) * px_y + px_y/2))
            picpoint2_x =  int(round(segment.points[1].x/(x_picmax - x_picmin) * px_x))
            picpoint2_y =  int(round(segment.points[1].y/(y_picmax - y_picmin) * px_y + px_y/2))

            #cv2.line(img,(picpoint1_y,picpoint1_x),(picpoint2_y,picpoint2_x),(255,255,255),10)





            d_i,phi_i,l_i = self.generateVote(segment)
            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue
            i = int(floor((d_i - self.d_min)/self.delta_d))
            j = int(floor((phi_i - self.phi_min)/self.delta_phi))
            measurement_likelihood[i,j] = measurement_likelihood[i,j] +  1
        # bridge = CvBridge()
        # img = self.rotateImage(img, self.phi_last/2.0/3.14*360)
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img1 = np.float32(img / 255.0) # img1.dtype = float32
        # dft = cv2.dft(img1, flags = cv2.DFT_COMPLEX_OUTPUT);
        # dft_shift = np.fft.fftshift(dft)
        # dft_magn = 20*np.log(cv2.magnitude(dft_shift[:,:,0],dft_shift[:,:,1]))
        #
        #
        # img2 = bridge.cv2_to_imgmsg(dft_magn.astype('uint8'), "mono8")
        # img2.header.stamp.secs = 0
        # img2.header.stamp.nsecs = 0
        # self.pub_img.publish(img2)
        #
        # shape = np.shape(dft_magn)
        #
        # c = 0
        #
        # for yi in range(0, shape[0]/2):
        #     for xi in range(0, shape[1]/2):
        #         c += dft_magn[yi, xi]
        #
        # c2 = 0
        #
        # for yi in range(0, shape[0]/2):
        #     for xi in range(shape[1]/2, shape[1]):
        #         c2 += dft_magn[yi, xi]
        #
        # print c2/c

        if np.linalg.norm(measurement_likelihood) == 0:
            return None
        measurement_likelihood = measurement_likelihood/np.sum(measurement_likelihood)
        return measurement_likelihood

    def rotateImage(self, image, angle):
        (h, w) = image.shape[:2]
        center = (w / 2, h / 2)
        M = cv2.getRotationMatrix2D(center,angle,1.0)
        rotated_image = cv2.warpAffine(image, M, (w,h))
        return rotated_image

    def getEstimate(self):
        maxids = np.unravel_index(self.belief.argmax(),self.belief.shape)
        # add 0.5 because we want the center of the cell
        d_max = self.d_min + (maxids[0]+0.5)*self.delta_d
        phi_max = self.phi_min + (maxids[1]+0.5)*self.delta_phi
        self.phi_last = phi_max
        return [d_max,phi_max]

    def getMax(self):
        return self.belief.max()

    def initialize(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        self.belief=RV.pdf(pos)


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
