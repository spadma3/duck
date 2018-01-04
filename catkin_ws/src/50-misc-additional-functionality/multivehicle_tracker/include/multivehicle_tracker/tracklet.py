from math import sqrt, atan2, atan
from scipy.spatial.distance import euclidean

import math
from kalman_filter import KalmanFilter


class Tracklet:
    STATUS_BORN = 0
    STATUS_TRACKING = 1
    STATUS_LOST = 2

    def __init__(self, global_id, x, y, confidence, timestamp):
        self.id = global_id
        self.last_update = timestamp
        self.estimator = KalmanFilter(x, y, 1 - confidence)
        self.status = Tracklet.STATUS_BORN

    def get_tracklet_info(self):
        return {
            'id' : self.id,
            'x'  : self.estimator.mu[0],
            'y'  : self.estimator.mu[2],
            'v'  : sqrt(self.estimator.mu[1] ** 2 + self.estimator.mu[3] ** 2),
            'phi': atan2(self.estimator.mu[1], self.estimator.mu[3]),
            'sigma_x' : self.estimator.sigma[0][0],
            'sigma_y' : self.estimator.sigma[2][2],
            'status': self.status
        }

    def estimate_position_at(self, timestamp):
        estimate = self.estimator.estimate_at(timestamp - self.last_update)[0]
        return estimate[0], estimate[2]

    def predict(self, timestamp):
        self.estimator.predict(math.fabs(self.last_update - timestamp))
        self.last_update = timestamp

    def update(self, detection, confidence):
        self.estimator.correct(detection, 1 - confidence)

    def similarity(self, coordinates, timestamp):
        return euclidean(self.estimate_position_at(timestamp), coordinates)

    def confidence(self):
        return self.estimator.sigma[0][0] * self.estimator.sigma[2][2]