import rospy
from math import *


class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.8
        self.z = controller.get_position().z
        self.x_ref = 0.1
        self.y_ref = 0.01
        self.x = controller.get_position().x
        self.time_start = self.controller.get_time()
        self.log_model = open('log/%s_regulation' % self.time_start, 'w')
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        roll, pitch, yaw = 0, 0, 0

        k_z = 2.# 1.5
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

        Az = vz * (k_z + alpha) + k_z * alpha * (z_gr - self.z_ref) - G
        Ax = vx * (k_x + betta) + k_x * betta * (x_gr - self.x_ref)
        Ay = vy * (k_x + betta) + k_x * betta * (y_gr - self.y_ref)

        Azz = Az
        Axx = Ax
        Ayy = Ay

        M1 = 0.42
        norm_coef = 1. * 0.45 / (M1 * G)
        thrust = norm_coef * (M1 * sqrt(Azz * Azz))
        #thrust = norm_coef * (M1 * sqrt(Azz * Azz + Axx * Axx))
       # pitch = 10.0 * atan(Axx / Azz)
       # roll = -10.0 * atan(Ayy / Azz)

        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f x %.2f vx %.2f' % (thrust, z_gr, vz, x_gr, vx))
        self.log_model.write("%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (self.time(), thrust, z_gr, vz, pitch, x_gr, vx, roll, y_gr, vy, yaw, Axx, Ayy, Azz))
        self.log_model.flush()

        self.controller.set_control(roll, pitch, yaw, thrust)

    def time(self):
        return self.controller.get_time() - self.time_start
