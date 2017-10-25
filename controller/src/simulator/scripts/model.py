from math import *
# from random import random, gauss

M1 = 2.2     # kg\n",
M2 = 0.
G = 9.81     # m/c^2\n",
L = 0.7      # m\n",
# MG = M * G
Ixx = 0.167
Iyy = 0.167  # + (M1+M2)*(3./13.)*(3./13.)*L*L\n",
Izz = 1.


class Model:

    def __init__(self, x=0., z=0., theta=0., gamma=0.):
        # init state
        self.x = x
        self.z = z
        self.theta = theta
        self.gamma = gamma

        # init speed
        self.dx = 0.
        self.dz = 0.
        self.dtheta = 0.
        self.dgamma = 0.

    def step(self, u3, u1, dt):

        u1 = 45.0 * u1
        # u1 = G*(M1+M2)
        # d2theta = u3 / Iyy
        # self.dtheta += d2theta * dt
        self.dtheta = 10. * (u3 - self.theta)  # !!! u3 is theta_ref
        self.theta += self.dtheta * dt

        mu1 = 0.

        # d2gamma = (M2+M1)/M1*((-*G/L)*(sin(self.gamma) - 1)-u1*(sin(self.theta) + cos(self.theta))/((M1+M2)*L))
        d2gamma = (-u1*sin(self.theta + self.gamma) + mu1*self.dx*abs(self.dx)*cos(self.gamma))/(M1*L)
        #d2gamma = 0.
        self.dgamma += d2gamma * dt
        self.gamma += self.dgamma * dt

        d2x = (u1*sin(self.theta)+M2*L*(sin(self.gamma)*self.dgamma*self.dgamma - cos(self.gamma)*d2gamma)- mu1*self.dx*abs(self.dx))/(M1+M2)
        self.dx += d2x * dt
        self.x += self.dx * dt

        d2z = (u1*cos(self.theta) - (M1+M2)*G + M2*L*(-cos(self.gamma)*self.dgamma*self.dgamma - sin(self.gamma)*d2gamma))/(M1+M2)
        self.dz += d2z * dt
        self.z += self.dz * dt
