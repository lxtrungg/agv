#!/usr/bin/env python3

import numpy as np
from math import atan2
from numpy import sin, cos
from numpy.random import multivariate_normal
class FusionEnKF(object):
    def __init__(self, dim_x, dim_z, size):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.N = size
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        self.K = np.zeros((dim_x, dim_z))

        self.sigmas = np.zeros((size, dim_x))
        self.sigmas_h = np.zeros((size, dim_z))

    def predict_update(self, z, u, dt):
        N = self.N

        # predict step
        for i, s in enumerate(self.sigmas):
            self.sigmas[i] = self.fx(s, u, dt)
        self.sigmas += multivariate_normal(np.zeros(self.dim_x), self.Q, size=N)
        x = np.mean(self.sigmas, axis=0)
        # P = self.outer_product(self.sigmas, x, self.residual_x)
        P = self.outer_product_sum(self.sigmas - x)
        P /= N - 1
        # update step
        for i, s in enumerate(self.sigmas):
            self.sigmas_h[i] = self.hx(s)

        zp = np.mean(self.sigmas_h, axis=0)
        # Pz = self.outer_product(self.sigmas_h, zp, self.residual_z)
        Pz = self.outer_product_sum(self.sigmas_h - zp)
        Pz = Pz / (N - 1) + self.R

        # Pxz = self.cross_variance(x, zp) / (N-1)
        Pxz = self.outer_product_sum(self.sigmas - x, self.sigmas_h - zp)
        Pxz /= (N - 1)
        
        self.K = Pxz @ np.linalg.inv(Pz)

        vr = multivariate_normal(np.zeros(self.dim_z), self.R, size=N)
        for i in range(N):
            y = self.residual_z(z + vr[i], self.sigmas_h[i])
            self.sigmas[i] += self.K @ y

        self.x = np.mean(self.sigmas, axis=0)
        # self.x[2] = atan2(sin(self.x[2]), cos(self.x[2]))
        self.P = P - self.K @ Pz @ self.K.T

        return self.x, self.P

    def outer_product(self, sigmas, x, residual_fn):
        P = 0
        for s in sigmas:
            y = residual_fn(s, x)
            P += np.outer(y, y)
        return P

    def outer_product_sum(self, a, b=None):
        if b is None:
            b = a
        outer = np.einsum('ij,ik->ijk', a, b)
        return np.sum(outer, axis=0)

    def cross_variance(self, x, z):
        sigmas = self.sigmas
        sigmas_h = self.sigmas_h
        Pxz = 0
        for i in range(self.N):
            dx = self.residual_x(sigmas[i], x)
            dz = self.residual_z(sigmas_h[i], z)
            Pxz += np.outer(dx, dz)
        return Pxz

    def fx(self, x, u, dt):
        theta = x[2]
        v = u[0]
        w = u[1]
        fxu = np.array([v*dt*cos(theta + 0.5*w*dt),
                        v*dt*sin(theta + 0.5*w*dt),
                        w*dt])
        return x + fxu

    def hx(self, x):
        if self.dim_z == 1:
            return x[[2]]
        elif self.dim_z == 2:
            return x[[0, 1]]
        else:
            return x

    def residual_x(self, a, b):
        y = a - b
        # y[2] = self.normallize_angle(y[2])
        return y

    def residual_z(self, a, b):
        y = a - b
        # if self.dim_z == 1:
        #     y = self.normallize_angle(y)
        # elif self.dim_z == 3:
        #     y[2] = self.normallize_angle(y[2])
        return y

    def normallize_angle(self, x):
        x %= (2*np.pi)
        if x > np.pi:
            x -= 2 * np.pi
        return x
        