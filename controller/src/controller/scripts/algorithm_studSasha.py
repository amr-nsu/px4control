import rospy
from math import *


class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.8 #0.8
        self.z = controller.get_position().z
        self.x_ref = -0.2
        self.y_ref =  0.2

        self.delta = 0.0  # was 0.4
        self.counter = 0

        self.x = controller.get_position().x
        self.time_start = self.controller.get_time()
        self.log_model = open('log/%s_regulation' % self.time_start, 'w')
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        roll, pitch, yaw = 0, 0, 0

        k_z = 1.5# 1.5
        alpha = k_z
        k_x = 0.3
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
        self.x_ref = 0. + self.delta
        self.counter += 1

        Az = vz * (k_z + alpha) + k_z * alpha * (z_gr - self.z_ref) - G
        Ax = vx * (k_x + betta) + k_x * betta * (x_gr - self.x_ref)
        Ay = vy * (k_x + betta) + k_x * betta * (y_gr - self.y_ref)

        Azz = Az
        Axx = Ax
        Ayy = Ay

        M1 = 0.6 #0.42
        norm_coef = 1. * M1 / (M1 * G) #0.45
        #thrust = norm_coef * (M1 * sqrt(Azz * Azz))
        thrust = norm_coef * (M1 * sqrt(Azz * Azz + Axx * Axx))
        #thrust = 2.
        pitch = 10.0 * atan(Axx / Azz)
        roll = -10.0 * atan(Ayy / Azz)

        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f z_ref %.2f x %.2f vx %.2f x_ref %.2f y_gr %.2f vy %.2f y_ref %.2f' % (thrust, z_gr, vz, self.z_ref, x_gr, vx, self.x_ref, y_gr, vy, self.y_ref))

        self.log_model.write("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (self.time(), thrust, z_gr, vz, pitch, x_gr, vx, roll, y_gr, vy, yaw, Axx, Ayy, Azz, self.z_ref, self.x_ref, self.y_ref))

        self.log_model.flush()

        self.controller.set_control(roll, pitch, yaw, thrust)

    def time(self):
        return self.controller.get_time() - self.time_start
