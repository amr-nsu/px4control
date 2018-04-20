# coding: utf-8

import math
from numpy import *
from random import gauss
from libstd import angle_to_pi

#from libstd import fetch_angle_to_2pi
PRINT = 0



class Kalman_Gamma:
    def __init__(self, x0 =0., y0 = 0.):        



        self.R = 0.01 * eye(6)
	self.R[4,4] = 0.001 
	self.R[5,5] = 0.001

        self.Q = 0.001 * eye(8)

        self.I = eye(8)


    def step(self, x_estimated, covariances, x_current, x_prediction, control, params, delta_time):


        m1, m2, Iyy, l = params

# ШАГ 1 - ПРЕДСКАЗАНИЕ


	theta, gamma = x_estimated[4], x_estimated[5]
        theta_dot, gamma_dot = x_estimated[6], x_estimated[7]
        

        u1, u3 = control


        F = array([[1.,   0.,  delta_time,   0.,  0., 0.,   0.,  0.],
                   [0.,   1.,  0.,   delta_time,  0., 0.,   0.,  0.],
                   [0.,   0.,  1.,   0., u1/(m1+m2)*delta_time*((1.+0.5*m2/m1)*cos(theta)+0.5*m2/m1*cos(theta+2.*gamma)), u1/(m1+m2)*delta_time*m2/m1*cos(theta+2.*gamma) + m2/(m1+m2)*l*(gamma_dot**2)*cos(gamma), 0., m2*l*2.*gamma_dot*sin(gamma)],   # initialize 2-d array
                   [0.,   0.,  0.,   1., u1/(m1+m2)*delta_time*(-(1.+0.5*m2/m1)*sin(theta)+0.5*m2/m1*sin(theta+2.*gamma)), u1/(m1+m2)*delta_time*m2/m1*sin(theta+2.*gamma) + m2/(m1+m2)*l*(gamma_dot**2)*sin(gamma), 0, -m2*l*2.*gamma_dot*cos(gamma)],
                   [0.,   0.,  0.,   0.,   1.,  0., delta_time, 0.],
                   [0.,   0.,  0.,   0.,   0.,  1., 0., delta_time],
                   [0.,   0.,  0.,   0.,   0.,  0., 1.+u3/Iyy*delta_time, 0.],
                   [0.,   0.,  0.,   0.,   0.,  0., 0., 1.]], dtype = double)



        H = array([[1.,   0.,  0.,   0.,  0., 0.,   0.,  0.],
                   [0.,   1.,  0.,   0.,  0., 0.,   0.,  0.],
                   [0.,   0.,  1.,   0.,  0., 0.,   0.,  0.],
		   [0.,   0.,  0.,   1.,  0., 0.,   0.,  0.],
		   [0.,   0.,  0.,   0.,  1., 0.,   0.,  0.],
		   [0.,   0.,  0.,   0.,  0., 1.,   0.,  0.]], dtype = double)


#--------- предсказание для матрицы ковариаций-------------------------
        covariances = dot(F,dot(covariances,transpose(F))) + self.Q
#        covariances = dot(F,dot(covariances,transpose(F))) + dot(J,dot(self.Q,transpose(J)))


# ШАГ 2 - КОРРЕКЦИЯ


        PH = dot(covariances,transpose(H))

        HPH = dot(H,PH)

        #print 'HPH', HPH
        S = HPH + self.R

#new
        S = (S + transpose(S))*0.5

        #print 'S', S

        try:
            L = linalg.cholesky(S).T
            Linv = linalg.inv(L)
            U1 = linalg.lstsq(L,eye(L.shape[1]))
            U2 = linalg.lstsq(L.T.conj(),eye(L.T.conj().shape[1]))
            W1 = dot(PH, U1[0])
            K = dot(W1, U2[0])
        except Exception as e:
            print "Odnako fignya...\n"
         #   print x_estimated, covariances, S, PH
            print e
            1./0.
            return x_estimated, covariances


#new

#usual
#        K = dot(PH,linalg.inv(S)) #матрица коэфф-а усиления
#        K = 0.*K


        #print 'K:=', K
#    print 'X:=', X

        if PRINT: print 'x', x_estimated

        x_estimated = x_estimated.reshape(len(x_estimated),1)

#        print 'z pred delta_z', features_d_a, features_prediction_d_a, features_d_a - features_prediction_d_a

        xx_delta = x_current - x_prediction
#        print x_prediction, dot(H1,x_prediction)
#        xx_delta = x_current - dot(H,x_prediction)
#        xx_delta[6] = angle_to_pi(xx_delta[6][0]) #vrode ne vliyaet no ostavim na vsyakii


        x_delta = dot(K,(xx_delta))

       # print 'x_current - x_prediction x_delta', x_current - x_prediction, x_delta

#        print 'x_old', x_estimated
        
        x_estimated = x_estimated + x_delta #коррекция вектора состояния с учетом измеренного выхода
#        x_estimated = x_estimated
        if PRINT: print 'x_new', x_estimated
#        print 'x_new', x_estimated

        x_estimated = x_estimated.reshape(1,len(x_estimated))[0]

#        self.x_robot_estimated = x_estimated[0:3]



#        covariances = dot((self.I - dot(K,H)),covariances) #обновление матрицы ковариаций

#        print 'covariances', covariances

#new
        covariances = covariances - dot(W1,transpose(W1))

#        covariances = covariances - dot(U1[0],U1[0].conj().T) #обновление матрицы ковариаций

#        x_features_estimated = delete(x_estimated,range(0,3),0)
#        x_estimated[2] = fetch_angle_to_2pi(x_estimated[2])

        return x_estimated, covariances


