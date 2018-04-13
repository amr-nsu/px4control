#coding: UTF8
import rospy
from math import *

import model
from kalman_quadro import Kalman
from numpy import *



from controller import saturation

class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 1.2 #0.8
        self.z = controller.get_position().z
        self.x_ref = -0.7 #-0.2
        self.y_ref =  0.2

        self.delta = 0.3  # was 0.4
        self.counter = 0
 	self.time_prev = 0.

#-------- for filtering vision position, getting of vision velosities
        self.x_cam = 0.
        self.y_cam = 0.
        self.z_cam = 0.
        self.x_cam_dot = 0.
        self.y_cam_dot = 0.
        self.z_cam_dot = 0.

	self.x_load = 0.
	self.y_load = 0.
	self.z_load = 0.
	self.x_load_dot = 0.
	self.y_load_dot = 0.
	self.z_load_dot = 0.
	
	self.gamma = 0.
	self.gamma_dot = 0.


        self.d_dxy = 0.8
        self.tau_dxy = 0.01
        self.k2_dxy = 1./(self.tau_dxy**2.)
        self.k_dxy = 1. * self.k2_dxy
        self.k1_dxy = 2.*self.d_dxy*self.tau_dxy*self.k2_dxy


        self.model = model.Model()
        self.model_ref = model.Model()

        self.kalman = Kalman()
        self.x_estimated = array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.], dtype = double)
        self.covariances = 0.0*eye(13)

        self.control = [0.,0.,0.,0.]


        self.delay1 = 15 #10
        self.u1_q = model.Variable_With_Delay(delay=self.delay1)
        self.u2_q = model.Variable_With_Delay(delay=self.delay1)
        self.u3_q = model.Variable_With_Delay(delay=self.delay1)
        self.u4_q = model.Variable_With_Delay(delay=self.delay1)


        self.time_start = self.controller.get_time()
        self.log_model = open('log/%s_regulation' % self.time_start, 'w')
        self.log_kalman = open('log/%s_kalman' % self.time_start, 'w')
        self.log_vision = open('log/%s_vision' % self.time_start, 'w')

        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
	dt = self.time() - self.time_prev

        roll, pitch, yaw = 0, 0, 0

#---------------------------------------------------------

        x_cam = self.controller.get_vision_position().x
        y_cam = self.controller.get_vision_position().y
        z_cam = self.controller.get_vision_position().z

	x_load = self.controller.get_vision_position_load().x
	y_load = self.controller.get_vision_position_load().y
	z_load = self.controller.get_vision_position_load().z

#----------2 degrees filter for estimating derivatives for x, y, z
        self.x_cam_dot += (-self.k1_dxy * self.x_cam_dot - self.k2_dxy * self.x_cam + self.k_dxy * x_cam) * dt
        self.x_cam += self.x_cam_dot * dt
        self.y_cam_dot += (-self.k1_dxy * self.y_cam_dot - self.k2_dxy * self.y_cam + self.k_dxy * y_cam) * dt
        self.y_cam += self.y_cam_dot * dt
        self.z_cam_dot += (-self.k1_dxy * self.z_cam_dot - self.k2_dxy * self.z_cam + self.k_dxy * z_cam) * dt
        self.z_cam += self.z_cam_dot * dt

	self.x_load_dot += (-self.k1_dxy * self.x_load_dot - self.k2_dxy * self.x_load + self.k_dxy * x_load) * dt
        self.x_load += self.x_load_dot * dt
        self.y_load_dot += (-self.k1_dxy * self.y_load_dot - self.k2_dxy * self.y_load + self.k_dxy * y_load) * dt
        self.y_load += self.y_load_dot * dt
        self.z_load_dot += (-self.k1_dxy * self.z_load_dot - self.k2_dxy * self.z_cam + self.k_dxy * z_cam) * dt
        self.z_load += self.z_load_dot * dt
	
	gamma = 0.
	if (z_load - z_cam) != 0:
		gamma = atan((x_cam - x_load)/(z_load - z_cam))
	self.gamma_dot += (-self.k1_dxy * self.gamma_dot - self.k2_dxy * self.gamma + self.k_dxy * gamma) * dt
	self.gamma += self.gamma_dot * dt

	phi_load = self.controller.get_euler_load()[0]
	theta_load = self.controller.get_euler_load()[1]
	psi_load = self.controller.get_euler_load()[2]

        #rospy.loginfo('vision filtered pos->(%.2f %.2f %.2f)->(%.2f %.2f %.2f) vel->(%.2f %.2f %.2f)' % (x_cam, y_cam, z_cam, self.x_cam, self.y_cam, self.z_cam, self.x_cam_dot, self.y_cam_dot, self.z_cam_dot))
        self.log_vision.write('%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n' \
                % (self.time(), x_cam, y_cam, z_cam, self.x_cam, self.y_cam, self.z_cam, self.x_cam_dot, self.y_cam_dot, self.z_cam_dot, self.x_load, self.y_load, self.z_load, phi_load, theta_load, psi_load, gamma, self.gamma, self.gamma_dot))
        self.log_vision.flush()


        w = self.controller.get_orientation().w
        x = self.controller.get_orientation().x
        y = self.controller.get_orientation().y
        z = self.controller.get_orientation().z

	

        phi = self.controller.get_euler()[0]
        theta = self.controller.get_euler()[1]
        psi = self.controller.get_euler()[2]

        #rospy.loginfo('orientation quat->(%.2f %.2f %.2f %.2f) euler->(%.2f %.2f %.2f)' % (w, x, y, z, phi, theta, psi))


        x_current = array([self.x_cam, self.y_cam, self.z_cam, self.x_cam_dot, self.y_cam_dot, self.z_cam_dot, psi, phi, theta], dtype = double)

#        x_current = array([self.x_cam, self.y_cam, self.z_cam, self.controller.get_linear_velocity().x, self.controller.get_linear_velocity().y, self.controller.get_linear_velocity().z, psi, phi, theta], dtype = double)


        self.model_ref.x, self.model_ref.y, self.model_ref.z, self.model_ref.dot.x, self.model_ref.dot.y, self.model_ref.dot.z, self.model_ref.psi, self.model_ref.phi, self.model_ref.theta = x_current
        self.model_ref.dot.psi, self.model_ref.dot.phi, self.model_ref.dot.theta = self.model.dot.psi, self.model.dot.phi, self.model.dot.theta

        n = 0
        while n < self.delay1:# delay additional steps
            self.model_ref.set_control([self.u1_q.get_index(self.delay1-n),self.u2_q.get_index(self.delay1-n),self.u3_q.get_index(self.delay1-n),self.u4_q.get_index(self.delay1-n)])
            self.model_ref.step(0.01)
            n += 1

        x_current_pred = array([self.model_ref.x, self.model_ref.y, self.model_ref.z, self.model_ref.dot.x, self.model_ref.dot.y, self.model_ref.dot.z, self.model_ref.psi, self.model_ref.phi, self.model_ref.theta], dtype = double)


#-------use prediction first

        x_prediction = array([self.model.x, self.model.y, self.model.z, self.model.dot.x, self.model.dot.y, self.model.dot.z, self.model.psi, self.model.phi, self.model.theta], dtype = double)

        self.x_estimated[0] = self.model.x
        self.x_estimated[1] = self.model.y
        self.x_estimated[2] = self.model.z
        self.x_estimated[3] = self.model.dot.x
        self.x_estimated[4] = self.model.dot.y
        self.x_estimated[5] = self.model.dot.z
        self.x_estimated[6] = self.model.psi
        self.x_estimated[7] = self.model.phi
        self.x_estimated[8] = self.model.theta
        self.x_estimated[9] = self.model.dot.psi
        self.x_estimated[10] = self.model.dot.phi
        self.x_estimated[11] = self.model.dot.theta

        self.x_estimated[12] = 1.


        self.x_estimated, self.covariances = self.kalman.step(self.x_estimated, self.covariances, x_current_pred.reshape(9,1), x_prediction.reshape(9,1), self.control, [self.model.k1_psi, self.model.k1_phi, self.model.k1_theta, self.model.k2_psi, self.model.k2_phi, self.model.k2_theta, 0], dt)


        #rospy.loginfo('kalman estimated pos->(%.2f %.2f %.2f %.2f %.2f %.2f) vel->(%.2f %.2f %.2f)' % (self.x_estimated[0], self.x_estimated[1], self.x_estimated[2], self.x_estimated[6], self.x_estimated[7], self.x_estimated[8], self.x_estimated[3], self.x_estimated[4], self.x_estimated[5]))

        self.log_kalman.write('%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n' \
                % (self.time(), self.x_estimated[0], self.x_estimated[1], self.x_estimated[2], self.x_estimated[3], self.x_estimated[4], self.x_estimated[5], self.x_estimated[6], self.x_estimated[7], self.x_estimated[8], self.x_estimated[9], self.x_estimated[10], self.x_estimated[11], self.x_estimated[12], psi, phi, theta))
        self.log_kalman.flush()

#---------------------------------------------------------

        k_z = 1.#2 1.5
        alpha = k_z
        k_x = 1.#2 0.3
        betta = k_x
        G = 9.81

#        x_gr = self.controller.get_position().x
#        y_gr = self.controller.get_position().y
#        z_gr = self.controller.get_position().z

        vx1 = self.controller.get_linear_velocity().x
        vy1 = self.controller.get_linear_velocity().y
        vz1 = self.controller.get_linear_velocity().z


        x_gr, y_gr, z_gr = self.x_estimated[0:3]
        vx, vy, vz = self.x_estimated[3:6]


	#print 'x_load ',x_load
	#print 'y_load ',y_load
	#print 'z_load ',z_load
 
	
        if self.counter % 1000 < 1:
            self.delta *= -1.
        # self.z_ref = 0.8 + self.delta
#        self.x_ref = 0. + 0.*self.delta


        self.counter += 1

        Az = vz * (k_z + alpha) + k_z * alpha * (z_gr - self.z_ref) - G
        Ax = vx * (k_x + betta) + k_x * betta * (x_gr - self.x_ref)
        Ay = vy * (k_x + betta) + k_x * betta * (y_gr - self.y_ref)

        Azz = Az
        Axx = Ax
        Ayy = Ay

        M1 = 0.8 #0.5 - без груза #0.55 #0.42
        #norm_coef = 1. * M1 / (M1 * G) #0.45
        #thrust = norm_coef * (M1 * sqrt(Azz * Azz))
        thrust = (M1 * sqrt(Azz * Azz + Axx * Axx + Ayy*Ayy))/G


        pitch = saturation(1.0 * atan(Axx / Azz),-0.06,0.06)
        roll =  saturation(-1.0 * atan(Ayy / Azz),-0.06,0.06)

#------------------------- one step prediction of states

        self.control = thrust*G, roll, pitch, 0.        # trust*G - because in model without normalization

        self.u1_q.value = self.control[0]
        self.u2_q.value = self.control[1]
        self.u3_q.value = self.control[2]
        self.u4_q.value = self.control[3]



        self.model.x, self.model.y, self.model.z, self.model.dot.x, self.model.dot.y, self.model.dot.z, self.model.psi, self.model.phi, self.model.theta, self.model.dot.psi, self.model.dot.phi, self.model.dot.theta, time_delay = self.x_estimated
        self.model.set_control(self.control)
        self.model.step(dt)



#        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f z_ref %.2f x %.2f vx %.2f x_ref %.2f y_gr %.2f vy %.2f y_ref %.2f ' % (thrust, z_gr, vz, self.z_ref, x_gr, vx, self.x_ref, y_gr, vy, self.y_ref))

        self.log_model.write("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (self.time(), thrust, z_gr, vz, pitch, x_gr, vx, roll, y_gr, vy, yaw, Axx, Ayy, Azz, self.z_ref, self.x_ref, self.y_ref, vx1, vy1, vz1))

        self.log_model.flush()


      #  if self.delta > 0:
      #      thrust = self.delta
      #  else:
       #     thrust = 0.

      #  if thrust > 0:
     #       thrust = 0.1



        self.controller.set_control(roll, pitch, yaw, thrust)
#       self.controller.set_actuator_control([1.*thrust, 0.*thrust, 0.*thrust, 0.*thrust, 1.*thrust, 1.*thrust, 1.*thrust, 1.*thrust])

        self.time_prev = self.time()

    def time(self):
        return self.controller.get_time() - self.time_start
