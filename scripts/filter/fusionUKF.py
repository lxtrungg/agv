#!/usr/bin/env python3

import numpy as np
from sympy import symbols, Matrix
from sympy import atan2
from numpy import sin, cos
# from math import sin, cos, atan2
class FusionUKF(object):
    def __init__(self, dim_x, dim_z, points):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        self.K = np.zeros((dim_x, dim_z))

        self.points = points
        self.num_sigmas = points.num_sigmas()
        self.Wm = points.Wm
        self.Wc = points.Wc

        self.sigmas_f = np.zeros((self.num_sigmas, dim_x))
        self.sigmas_h = np.zeros((self.num_sigmas, dim_z))

    def predict_update(self, z):
        # predict step
        x, P = self.unscented_transform(self.sigmas_f, self.Q, self.x_mean_fn, self.residual_x)

        # update step
        for i, s in enumerate(self.sigmas_f):
            self.sigmas_h[i] = self.hx(s)
        zp, Pz = self.unscented_transform(self.sigmas_h, self.R, self.z_mean_fn, self.residual_z)
        Pxz = self.cross_variance(x, zp)

        self.K = Pxz @ np.linalg.inv(Pz)
        y = self.residual_z(z, zp)

        self.x = x + self.K @ y
        # self.x[2] = atan2(sin(self.x[2]), cos(self.x[2]))
        self.P = P - self.K @ Pz @ self.K.T

        return self.x, self.P

    def compute_process_sigmas(self, x, P, u, dt):
        sigmas = self.points.sigma_points(x, P)
        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = self.fx(s, u, dt)

    def unscented_transform(self, sigmas, noise_cov, mean_fn, residual_fn):
        kmax, n = sigmas.shape

        x = mean_fn(sigmas, self.Wm)
        P = np.zeros((n, n))
        for k in range(kmax):
            y = residual_fn(sigmas[k], x)
            P += self.Wc[k] * np.outer(y, y)
        P += noise_cov
        return (x, P)


    def cross_variance(self, x, z):
        sigmas_f = self.sigmas_f
        sigmas_h = self.sigmas_h
        Pxz = np.zeros((sigmas_f.shape[1], sigmas_h.shape[1]))
        N = sigmas_f.shape[0]
        for i in range(N):
            dx = self.residual_x(sigmas_f[i], x)
            dz = self.residual_z(sigmas_h[i], z)
            Pxz += self.Wc[i] * np.outer(dx, dz)
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

    def x_mean_fn(self, sigmas, Wm):
        n = sigmas.shape[1]
        x = np.zeros(n)
        x[0] = np.sum(sigmas[:, 0] @ Wm)
        x[1] = np.sum(sigmas[:, 1] @ Wm)
        # sum_sin = np.sum(sin(sigmas[:, 2]) @ Wm)
        # sum_cos = np.sum(cos(sigmas[:, 2]) @ Wm)
        # x[2] = atan2(sum_sin, sum_cos)
        x[2] = np.sum(sigmas[:, 2] @ Wm)
        return x

    def z_mean_fn(self, sigmas, Wm):
        n = sigmas.shape[1]
        z = np.zeros(n)
        if n == 1:
            # sum_sin = np.sum(sin(sigmas[:, 0]) @ Wm)
            # sum_cos = np.sum(cos(sigmas[:, 0]) @ Wm)
            # z[0] = atan2(sum_sin, sum_cos)
            z[0] = np.sum(sigmas[:, 0] @ Wm)
        elif n == 2:
            z[0] = np.sum(sigmas[:, 0] @ Wm)
            z[1] = np.sum(sigmas[:, 1] @ Wm)
        else:
            z[0] = np.sum(sigmas[:, 0] @ Wm)
            z[1] = np.sum(sigmas[:, 1] @ Wm)
            # sum_sin = np.sum(sin(sigmas[:, 2]) @ Wm)
            # sum_cos = np.sum(cos(sigmas[:, 2]) @ Wm)
            # z[2] = atan2(sum_sin, sum_cos)
            z[2] = np.sum(sigmas[:, 2] @ Wm)

        return z

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
        