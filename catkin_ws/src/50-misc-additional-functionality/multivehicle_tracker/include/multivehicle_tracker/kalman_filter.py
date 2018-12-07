import numpy as np


class KalmanFilter:
    def __init__(self, x, y, e):
        self.mu = [x, 0.0, y, 0.0]
        self.sigma = np.diag([e, 1.0, e, 1.0])
        self.Q = np.diag([0.01, 0.01])
        self.H = np.array([[1, 0, 0, 0],
                          [0, 0, 1, 0]])
        self.R = np.diag([0.01, 0.01])
        self.I = np.eye(4)

    def F(self, dt):
        return np.array([
            [1.0,  dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0,  dt],
            [0.0, 0.0, 0.0, 1.0]
        ])

    def L(self, dt):
        return np.array([[0, 0],
                        [1, 0],
                        [0, 0],
                        [0, 1]]) * dt

    def G(self, dt):
        return np.array([
            [0.5 * dt ** 2, 0],
            [dt,              0],
            [0.0,   0.5 * dt ** 2],
            [0.0,              dt]
        ])

    def estimate_at(self, dt, u=np.array([0.0, 0.0])):
        F = self.F(dt)
        L = self.L(dt)
        G = self.G(dt)
        return np.matmul(F, self.mu) + np.matmul(G, u.T), np.matmul(F, np.matmul(self.sigma, F.T)) + np.matmul(L, np.matmul(self.Q, L.T))

    def predict(self, t, u=np.array([0.0, 0.0])):
        self.mu, self.sigma = self.estimate_at(t, u)

    def correct(self, point, e):
        z = np.array([point[0], point[1]])
        S = np.matmul(self.H, np.matmul(self.sigma, self.H.T)) + self.R
        K = np.matmul(self.sigma, np.matmul(self.H.T, np.linalg.inv(S)))
        y = z - np.matmul(self.H, self.mu)
        self.mu = self.mu + np.matmul(K, y)
        self.sigma = np.matmul(self.I - np.matmul(K, self.H), self.sigma)

