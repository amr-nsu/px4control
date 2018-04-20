from math import sin, cos, pi, sqrt
#from control_simple_square import *
#from control import *
#from control_real import *
from kalman_quadro import Kalman
from libstd import angle_to_pi

from numpy import *



m_real = 0.5

G = 9.81     # m/c^2
#Ixx = 1.
Iyy = 0.167
#Izz = 1.

delay1 = 0
delay = 0

phi_roll_delay = 0
yaw_delay = 0


#0.5 - 3 s
#0.2 - 0.75-1.0 s
#0.15 - same
#0.1 - 0.5 s

tau_common = 0.17 #0.15
tau_real = 0.17

d_common = 0.35 #0.8
d_real = 0.35

k2_common = 1. #0.8
k2_real = 1.

k_u0 = 1.
k_phi = 1.
k_psi = 1.
k_theta = 1.

class Variable_With_Delay(object):

    def __init__(self, init=0, delay=0):
        self.__query = [init] * (delay + 1)

    def set_value(self, new_value):
        tmp = self.__query[0:-1]
        self.__query = [new_value]
        self.__query.extend(tmp)

    def get_value(self):
        return self.__query[-1]

    def get_index(self, index):
        return self.__query[index]

    def get_last_value(self):
        return self.__query[0]

    value = property(get_value, set_value)
    last = property(get_last_value)



class Model:

    def __init__(self, x=0., z=0., theta=0., gamma = 0.):
#    def __init__(self, x=0., y=0., z=0., psi=0., phi=0., theta=0., tau_psi = 0.1, tau_phi = 0.1, tau_theta = 0.1, M = 0.4):

        # init state
        self.x = x
        self.z = z
	self.theta = theta
	self.gamma = gamma

        self.dx = 0.
        self.dz = 0.
	self.dtheta = 0.
	self.dgamma = 0.

        # init control
        self.u = (0., 0., 0., 0.)
	self.M1 = 0.6
	self.M2 = 0.2
	self.Iyy = 0.1
	self.L = 0.4


    def set_control(self, u):
        self.u = u

    def step(self, dt):
	
	d2theta = self.u[3]/self.Iyy
	self.dtheta += d2theta*dt
	self.theta += self.dtheta*dt

	d2gamma = (-self.u[1]*sin(self.theta+self.gamma))/(self.M1*self.L)
	self.dgamma += d2gamma*dt
	self.gamma +=self.dgamma*dt

	d2x = (self.u[1]*sin(self.theta) +self.M2*self.L*(sin(self.gamma)*self.dgamma*self.dgamma - cos(self.gamma)*d2gamma))/(self.M1+self.M2)
	self.dx += d2x*dt
	self.x += self.dx*dt

	d2z = (self.u[1]*cos(self.theta) - (self.M1+self.M2)*G + self.M2*self.L*(-cos(self.gamma)*self.dgamma*self.dgamma - sin(self.gamma)*d2gamma))/(self.M1+self.M2)
	self.dz += d2z*dt
	self.z += self.dz*dt




