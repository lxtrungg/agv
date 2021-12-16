#!/usr/bin/env python

import numpy as np
from scipy.linalg import cholesky

class SigmaPoints(object):
    def __init__(self, dim_x, alpha, beta, kappa):
        self.n = dim_x
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.sqrt = cholesky
        self.compute_weights()


    def num_sigmas(self):
        return 2*self.n + 1

    def sigma_points(self, x, P):
        n = self.n
        lambda_ = self.alpha**2 * (n + self.kappa) - n
        U = self.sqrt((lambda_ + n)*P)
        sigmas = np.zeros((2*n+1, n))
        sigmas[0] = x
        for k in range(n):
            sigmas[k+1]   = self.residual(x, -U[k])
            sigmas[n+k+1] = self.residual(x, U[k])
        return sigmas

    def compute_weights(self):
        n = self.n
        lambda_ = self.alpha**2 * (n + self.kappa) - n
        c = .5 / (n + lambda_)
        self.Wc = np.full(2*n + 1, c)
        self.Wm = np.full(2*n + 1, c)
        self.Wc[0] = lambda_ / (n + lambda_) + (1 - self.alpha**2 + self.beta)
        self.Wm[0] = lambda_ / (n + lambda_)

    def residual(self, a, b):
        y = a - b
        # y[2] = self.normallize_angle(y[2])
        # y = self.normallize_angle(y)
        return y

    def normallize_angle(self, x):
        x %= (2*np.pi)
        if x > np.pi:
            x -= 2 * np.pi
        return x


