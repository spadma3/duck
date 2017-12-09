from duckietown_utils.parameters import Configurable
from duckietown_msgs.msg import Segment
import numpy as np
from lane_filter.lane_filter_interface import LaneFilterInterface
import copy
from scipy.stats import multivariate_normal
from duckietown_utils import load_camera_intrinsics, rectify, d8_compressed_image_from_cv_image
import cv2, torch
from load_model import load_model



class LaneFilterPF(Configurable, LaneFilterInterface):
    """LaneFilterPF"""

    def __init__(self, robot_name, configuration):
        param_names = [
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'sigma_d',
            'sigma_phi',
            'n_particles',
            'lik_dist',
        ]
        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self,param_names,configuration)
        self.d_noise = lambda: np.random.normal(0, self.sigma_d, self.n_particles)
        self.phi_noise = lambda: np.random.normal(0, self.sigma_phi, self.n_particles)
        self.intrinsics = load_camera_intrinsics(robot_name)
        self.belief = np.zeros((20,20))
        self.initialize()
        self.model = load_model()
        self.obs = None


    def predict(self, dt, v, w):
        return
        dd = v*dt*np.sin(self.particles[:,1])
        dphi = w*dt
        self.particles[:,0] += dd + self.d_noise()
        self.particles[:,0] = np.maximum(self.d_min, self.particles[:,0])
        self.particles[:,0] = np.minimum(self.d_max, self.particles[:,0])
        self.particles[:,1] += dphi + self.phi_noise()
        self.particles[:,1] = np.maximum(self.phi_min, self.particles[:,1])
        self.particles[:,1] = np.minimum(self.phi_max, self.particles[:,1])

    def update(self, image):
        image = rectify(image, self.intrinsics)
        image = cv2.resize(image, (64,64))
        obs = self.get_obs((image / 128.) - 1.)
        self.obs = obs
        return

        lik = multivariate_normal.pdf(self.particles, mean=obs, cov=np.array([[.05 ** 2, 0.], [0., .05 ** 2]]))
        self.weights *= lik
        # check for divergence
        if np.sum(self.weights) < 1e-8:
            self.initialize()
            return
        self.weights /= np.sum(self.weights)
        Neff = 1. / np.sum(self.weights ** 2)
        if Neff < self.n_particles / 2.0:
            self.resample()

    def getEstimate(self):
        return self.obs
        #return self.particles[np.argmax(self.weights)]
        #return self.weights.dot(self.particles)

    def initialize(self):
        d = np.random.normal(0, .1, self.n_particles)
        phi = np.random.normal(0, .2, self.n_particles)
        self.particles = np.stack((d,phi), axis=1)
        self.weights = np.ones(self.n_particles) / self.n_particles

    def inLane(self):
        return np.abs(self.getEstimate()[0]) < self.lanewidth / 3.0

    def resample(self):
        inds = np.random.choice(np.arange(self.n_particles), self.n_particles, replace=True, p=self.weights)
        self.particles = self.particles[inds]
        self.weights = np.ones(self.n_particles) / self.n_particles

    def get_obs(self, image):
        image = np.float32(image.transpose(2,0,1)[None])
        image = torch.autograd.Variable(torch.from_numpy(image))
        obs = self.model(image).data.numpy()[0]
        obs[0] = -1.* (obs[0] - 1.12) # change from simulator coordinates to duckietown
        return obs
