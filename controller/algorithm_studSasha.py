import rospy
from math import *


class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.5
        self.z = controller.get_position().z
        self.time_start = self.controller.get_time()
        self.log_model = open('log/13_10 x_regulation','w')
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        # 0.45; P: 0.2; D: 0
       # thrust = 0.45
        #    + 0.1 * (self.z_ref - self.controller.get_position().z) \
         #   - 0.01 * (self.controller.get_linear_velocity().z)

        z_gr = self.controller.get_position().z
        dz_gr = self.controller.get_linear_velocity().z

        x_gr = self.controller.get_position().x
        dx_gr = self.controller.get_linear_velocity().x
        k_z = 3.
        alpha = k_z
        G = 9.81
        T = 0.1
#        vz = (self.controller.get_position().z - self.z)/T
        vz = self.controller.get_linear_velocity().z
        self.z = self.controller.get_position().z
        Az = vz*(k_z+alpha) + k_z*alpha*(z_gr - self.z_ref) - G
        Azz = Az

        M1 = 0.4
        norm_coef = 1.*0.45/(M1*G)
        thrust = norm_coef*(M1*sqrt(Azz*Azz))
        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f x %.2f dx_gr %.2f' % (thrust, z_gr, vz, x_gr, dx_gr))
        self.log_model.write("%f %f %f %f %f %f\n" % (self.time(), thrust, z_gr, vz, x_gr, dx_gr))
        self.log_model.flush()

        roll, pitch, yaw = 0, 0, 0
        self.controller.set_control(roll, pitch, yaw, thrust)

    def time(self):
        return self.controller.get_time() - self.time_start
