#!/usr/bin/env python3

import numpy as np
from sympy import symbols, Matrix, sin, cos, atan2

class FusionEKF(object):
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros((dim_x, 1))
        self.P = np.eye(dim_x)
        self.F = np.eye(dim_x)
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        self.S = np.zeros((dim_z, dim_z))
        self.K = np.zeros(self.x.shape)
        self.I = np.identity(dim_x)

        x, y, theta, v, w, t = symbols('x, y, theta, v, w, t')
        state = Matrix([x, y, theta])
        control = Matrix([v, w])

        self.fxu = Matrix([[x + v*t*cos(theta + 0.5*w*t)],
                           [y + v*t*sin(theta + 0.5*w*t)],
                           [theta + w*t]])
        self.F = self.fxu.jacobian(state)
        self.W = self.fxu.jacobian(control)
        
        if dim_z == 1:
            self.hx = Matrix([[theta]])
        elif dim_z == 2:
            self.hx = Matrix([[x], [y]])
        else:
            self.hx = Matrix([[x], [y], [theta]])
            
        self.H = self.hx.jacobian(state)
        self.subs = {x:0, y:0, theta:0, v:0, w:0, t:0}
        self.x_x = x
        self.x_y = y
        self.x_theta = theta
        self.u_v = v
        self.u_w = w
        self.t = t

    def predict_update(self, z, u, dt):
        self.subs[self.x_x] = self.x[0,0]
        self.subs[self.x_y] = self.x[1,0]
        self.subs[self.x_theta] = self.x[2, 0]
        self.subs[self.u_v] = u[0, 0]
        self.subs[self.u_w] = u[1, 0]
        self.subs[self.t] = dt

        F = np.array(self.F.evalf(subs=self.subs)).astype(float)
        W = np.array(self.W.evalf(subs=self.subs)).astype(float)
        H = np.array(self.H.evalf(subs=self.subs)).astype(float)
        Q = self.Q
        R = self.R
        I = self.I
        x = self.x
        P = self.P

        #predict step
        fxu = np.array(self.fxu.evalf(subs=self.subs)).astype(float)
        x = fxu
        P = F @ P @ F.T + W @ Q @ W.T
        
        #update step
        hx = np.array(self.hx.evalf(subs=self.subs)).astype(float)
        y = self.residual(z, hx)
        PHT = P @ H.T
        self.S = H @ PHT + R
        SI = np.linalg.inv(self.S)
        self.K = PHT @ SI
        self.x = x + self.K @ y
        # self.x[2, 0] = atan2(sin(self.x[2, 0]), cos(self.x[2, 0]))
        
        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
        I_KH = I - self.K @ H
        self.P = I_KH @ P @ I_KH.T + self.K @ R @ self.K.T
        
        return self.x

    def residual(self, a, b):
        y = a - b
        # if self.dim_z == 1:
        #     y = self.normallize_angle(y)
        # elif self.dim_z == 3:
        #     y[2, 0] = self.normallize_angle(y[2, 0])
        return y

    def normallize_angle(self, x):
        x %= (2*np.pi)
        if x > np.pi:
            x -= 2 * np.pi
        return x
