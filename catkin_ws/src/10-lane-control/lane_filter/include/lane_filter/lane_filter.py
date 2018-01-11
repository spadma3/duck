from duckietown_utils.parameters import Configurable
from duckietown_msgs.msg import Segment
import numpy as np
from .lane_filter_interface import LaneFilterInterface
from scipy.stats import multivariate_normal
from scipy.ndimage.filters import gaussian_filter
from math import floor, pi, sqrt
import copy



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
            'curvature_res',
            'range_min',
            'range_est',
            'range_max',
            'curvature_right',
            'curvature_left'
        ]
        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self,param_names,configuration)

        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.beliefArray = []
        self.range_arr = np.zeros(self.curvature_res + 1)
        for i in range(self.curvature_res + 1):
            self.beliefArray.append(np.empty(self.d.shape))
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0  = [ [self.sigma_d_0, 0], [0, self.sigma_phi_0] ]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        self.d_med_arr = []
        self.phi_med_arr = []
        self.median_filter_size = 5
        
        self.initialize()
        
    
    def predict(self, dt, v, w):
        delta_t = dt
        d_t = self.d + v*delta_t*np.sin(self.phi)
        phi_t = self.phi + w*delta_t

        for k in range(self.curvature_res):
            p_belief = np.zeros(self.beliefArray[k].shape)

            # there has got to be a better/cleaner way to do this - just applying the process model to translate each cell value
            for i in range(self.beliefArray[k].shape[0]):
                for j in range(self.beliefArray[k].shape[1]):
                    if self.beliefArray[k][i,j] > 0:
                        if d_t[i,j] > self.d_max or d_t[i,j] < self.d_min or phi_t[i,j] < self.phi_min or phi_t[i,j] > self.phi_max:
                            continue
                        i_new = int(floor((d_t[i,j] - self.d_min)/self.delta_d))
                        j_new = int(floor((phi_t[i,j] - self.phi_min)/self.delta_phi))
                        p_belief[i_new,j_new] += self.beliefArray[k][i,j]

            s_belief = np.zeros(self.beliefArray[k].shape)
            gaussian_filter(p_belief, self.cov_mask, output=s_belief, mode='constant')

            if np.sum(s_belief) == 0:
                return
            self.beliefArray[k] = s_belief / np.sum(s_belief)


    def prepareSegments(self, segments):
        segmentsRangeArray = []
        # emptyArr = []
        for i in range(len(self.range_arr)):
            segmentsRangeArray.append([])
        for segment in segments:
            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue

            # only consider points in a certain range from the Duckiebot
            point_range = self.getSegmentDistance(segment)
            if point_range < self.range_est:
                segmentsRangeArray[0].append(segment)
            if self.curvature_res is not 0:
                for i in range(self.curvature_res):
                    if point_range < self.range_arr[i+1] or point_range > self.range_arr[i]:
                        segmentsRangeArray[i + 1].append(segment)

        print 'Range Array values: %s' % range_arr
        for i in range(len(segmentsRangeArray)):
            print 'Length of segmentRangeArray[%i]: %i' % (i, len(segmentsRangeArray[i]))
            for i in range(len(segmentsRangeArray[i])):
                print 'Lenght of segment %i: %f' % (i, self.getSegmentDistance(segment))

        return segmentsRangeArray


    def updateRangeArray(self, curvature_res):
        self.curvature_res = curvature_res
        self.beliefArray = []
        for i in range(self.curvature_res + 1):
            self.beliefArray.append(np.empty(self.d.shape))
        self.initialize()
        if curvature_res > 1:
            self.range_arr = np.zeros(self.curvature_res + 1)
            range_diff = (self.range_max - self.range_min) / (self.curvature_res)
            
            # populate range_array
            for i in range(len(self.range_arr)):
                self.range_arr[i] = self.range_min + (i * range_diff)
        

    def update(self, segments):
        segmentsRangeArray = self.prepareSegments(segments)
        self.updatePoseBelief(segmentsRangeArray[0])
        if self.curvature_res > 0:
            for i in range(self.curvature_res):
                # print 'Updating beliefArray[%i]' % i
                self.updateCurvatureBelief(segmentsRangeArray[i + 1], i + 1)

            
    def updatePoseBelief(self, segments):
        measurement_likelihood = self.generate_measurement_likelihood(segments)
        if measurement_likelihood is not None:
            self.beliefArray[0] = np.multiply(self.beliefArray[0], measurement_likelihood)
    

    def updateCurvatureBelief(self, segments, index):
        measurement_likelihood = self.generate_measurement_likelihood(segments)
        if measurement_likelihood is not None:
            self.beliefArray[index] = np.multiply(self.beliefArray[index], measurement_likelihood)
            if np.sum(self.beliefArray[index]) == 0:
                self.beliefArray[index] = measurement_likelihood
            else:
                self.beliefArray[index] = self.beliefArray[index] / np.sum(self.beliefArray[index])


    def generate_measurement_likelihood(self, segments):
        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape)

        for segment in segments:
            d_i, phi_i, l_i = self.generateVote(segment)

            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue

            i = int(floor((d_i - self.d_min)/self.delta_d))
            j = int(floor((phi_i - self.phi_min)/self.delta_phi))
            measurement_likelihood[i,j] = measurement_likelihood[i,j] + 1

        if np.linalg.norm(measurement_likelihood) == 0:            
            return None
        measurement_likelihood = measurement_likelihood/np.sum(measurement_likelihood)
        return measurement_likelihood
        

    def getEstimate(self):
        d_max = np.zeros(self.curvature_res + 1)
        phi_max = np.zeros(self.curvature_res + 1)
        for i in range(self.curvature_res + 1):
            maxids = np.unravel_index(self.beliefArray[i].argmax(),self.beliefArray[i].shape)
            d_max[i] = self.d_min + (maxids[0] + 0.5)*self.delta_d
            phi_max[i] = self.phi_min + (maxids[1] + 0.5)*self.delta_phi
        return [d_max, phi_max]


    def getCurvature(self, d_max, phi_max):
        d_med_act = np.median(d_max)
        phi_med_act = np.median(phi_max)

        if len(self.d_med_arr) >= self.median_filter_size:
            self.d_med_arr.pop(0)
            self.phi_med_arr.pop(0)
        self.d_med_arr.append(d_med_act)
        self.phi_med_arr.append(phi_med_act)

        # set curvature
        # TODO: check magic constants
        if np.median(self.phi_med_arr) - phi_max[0] < -0.3 and np.median(self.d_med_arr) > 0.05:
            print "Curvature estimation: left curve"
            return self.curvature_left
        elif np.median(self.phi_med_arr) - phi_max[0] > 0.2 and np.median(self.d_med_arr) < -0.02:
            print "Curvature estimation: right curve"
            return self.curvature_right
        else:
            print "Curvature estimation: straight lane"
            return 0


    def getMax(self):
        return self.beliefArray[0].max()


    def initialize(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        for i in range(self.curvature_res + 1):
            self.beliefArray[i]=RV.pdf(pos)


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

