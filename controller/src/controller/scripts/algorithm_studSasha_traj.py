import rospy
from math import *

from controller import saturation

class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 1. #0.8
        self.z = controller.get_position().z
        self.x_ref = -0.2
        self.y_ref =  0.4

        self.delta = 0.  # was 0.4
        self.counter = 0
        self.circle_time = 0
        self.circle_period = 7.
        self.spiral_time = 0
        self.time_prev = 0.
        

        self.x = controller.get_position().x
        self.time_start = self.controller.get_time()
        self.log_model = open('log/%s_regulation' % self.time_start, 'w')
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        roll, pitch, yaw = 0, 0, 0

        k_z = 2.# 1.5
        alpha = k_z
        k_x = 2.#0.3
        betta = k_x
        G = 9.81
        # T = 0.1
        # vz = (self.controller.get_position().z - self.z) / T
        # self.z = self.controller.get_position().z

        x_gr = self.controller.get_position().x
        y_gr = self.controller.get_position().y
        z_gr = self.controller.get_position().z

        vx = self.controller.get_linear_velocity().x
        vy = self.controller.get_linear_velocity().y
        vz = self.controller.get_linear_velocity().z

        if self.counter % 1000 < 1:
            self.delta *= -1.
        # self.z_ref = 0.8 + self.delta
#        self.x_ref = 0. + 0.*self.delta
        
        dt = self.time() - self.time_prev

        if self.time() > 50 and self.circle_time < 7.*self.circle_period:
            x0 = -0.2
            y0 =  0.2
            R = 0.2
            self.x_ref = x0 + R*sin(2.*pi/self.circle_period*self.circle_time)
            self.y_ref = y0 + R*cos(2.*pi/self.circle_period*self.circle_time)
            self.circle_time += dt
        elif self.time() > 50+20+7.*self.circle_period and self.circle_time > 6.*self.circle_period and self.spiral_time < 3.*self.circle_period and self.z_ref>=0.2:
            x0 = -0.2
            y0 =  0.2
            R = 0.2
            self.x_ref = x0 + R*sin(2.*pi/self.circle_period*self.spiral_time)
            self.y_ref = y0 + R*cos(2.*pi/self.circle_period*self.spiral_time)
            self.z_ref += -0.8/(3.*self.circle_period)*dt
            self.spiral_time += dt
        elif self.time() > 50+20+7.*self.circle_period+3.*self.circle_period:
            self.x_ref = -0.2
            self.y_ref =  0.4
            self.z_ref =  0.2

        self.time_prev = self.time()

        self.counter += 1

        Az = vz * (k_z + alpha) + k_z * alpha * (z_gr - self.z_ref) - G
        Ax = vx * (k_x + betta) + k_x * betta * (x_gr - self.x_ref)
        Ay = vy * (k_x + betta) + k_x * betta * (y_gr - self.y_ref)

        Azz = Az
        Axx = Ax
        Ayy = Ay

        M1 = 0.5 #0.55 #0.42
        #norm_coef = 1. * M1 / (M1 * G) #0.45
        #thrust = norm_coef * (M1 * sqrt(Azz * Azz))
        thrust = (M1 * sqrt(Azz * Azz + Axx * Axx + Ayy*Ayy))/G

        #if self.delta > 0:
        #    thrust = self.delta
        #else:
        #    thrust = 0.

        pitch = saturation(1.0 * atan(Axx / Azz),-0.06,0.06)
        roll =  saturation(-1.0 * atan(Ayy / Azz),-0.06,0.06)

        
	#p1(-0.5,0.6) p2(0.,-0.63)

        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f z_ref %.2f x %.2f vx %.2f x_ref %.2f y_gr %.2f vy %.2f y_ref %.2f ' % (thrust, z_gr, vz, self.z_ref, x_gr, vx, self.x_ref, y_gr, vy, self.y_ref))

        self.log_model.write("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (self.time(), thrust, z_gr, vz, pitch, x_gr, vx, roll, y_gr, vy, yaw, Axx, Ayy, Azz, self.z_ref, self.x_ref, self.y_ref))

        self.log_model.flush()

        self.controller.set_control(roll, pitch, yaw, thrust)

    def time(self):
        return self.controller.get_time() - self.time_start
