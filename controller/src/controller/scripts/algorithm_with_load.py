import rospy
from math import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 1.8
        self.z = controller.get_position().z
        self.dx_ref = 0.1
        self.y_ref = 0.01
        self.gamma = 0.0
        self.counter = 0
        self.x = controller.get_position().x
        self.time_start = self.controller.get_time()
        self.log_model = open('log/%s_regulation' % self.time_start, 'w')
        self.controller.set_control_loop(0.01, self.loop)
        rospy.Subscriber('/odom', Odometry, self.__odometry_callback)

    def loop(self):
        roll, pitch, yaw = 0, 0, 0

        k_z = 1.5
        alpha = k_z
        alpha2 = 1.0
        k_gamma = 1.0
        k_x = 0.3
        betta = k_x
        G = 9.81
        # T = 0.1
        # vz = (self.controller.get_position().z - self.z) / T
        # self.z = self.controller.get_position().z
        L = 1.0
        M2 = 0.1
        M1 = 0.42

         
        if ((self.counter) % 50 == 0):
            self.dx_ref = -1.*self.dx_ref
            #theta_ref = -1*theta_ref
            
        gamma = self.gamma
        vgamma = 0 # FIXME
        gamma = 0
        
        # !!define gamma & vgamma & L & M2!! y??
        x_gr = self.controller.get_position().x + L * sin(gamma)
        y_gr = self.controller.get_position().y
        z_gr = self.controller.get_position().z - L * cos(gamma)

        vx = self.controller.get_linear_velocity().x + vgamma * L * cos(gamma)
        vy = self.controller.get_linear_velocity().y
        vz = self.controller.get_linear_velocity().z + vgamma * L * sin(gamma)

        Az = M1*L*vgamma*vgamma*cos(gamma)/(M1+M2) + vz * (k_z + alpha) + k_z * alpha * (z_gr - self.z_ref) - G
      #  Ax = -M1*L*vgamma*vgamma*sin(gamma)/(M1+M2) + vx * (k_x + betta) + k_x * betta * (x_gr - self.x_ref)
        Ax = -M1*L*vgamma*vgamma*sin(gamma)/(M1+M2) + betta*(dx_gr - self.dx_ref)
        Ay = vy * (k_x + betta) + k_x * betta * (y_gr - self.y_ref)

        Azz = Az  - 1.*sin(gamma)*M1*((alpha2+k_gamma)*vgamma+alpha2*k_gamma*gamma)/(M1+M2)
        Axx = Ax  - 1.*cos(gamma)*M1*((alpha2+k_gamma)*vgamma+alpha2*k_gamma*gamma)/(M1+M2)
        Ayy = Ay

        print Ax, Ay, Az

        norm_coef = 1. * 0.45 / (M1 * G)
        # thrust = norm_coef * (M1 * sqrt(Azz * Azz))
        thrust = norm_coef * (M1 * sqrt(Azz * Azz + Axx * Axx))
        pitch = 10.0 * atan(Axx / Azz)
        roll = -10.0 * atan(Ayy / Azz)

        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f x %.2f vx %.2f' % (thrust, z_gr, vz, x_gr, vx))
        self.log_model.write("%f %f %f %f %f %f %f %f %f %f %f %f %f %f\n" % (self.time(), thrust, z_gr, vz, pitch, x_gr, vx, roll, y_gr, vy, yaw, Axx, Ayy, Azz))
        self.log_model.flush()

        self.controller.set_control(roll, pitch, yaw, thrust)
        self.counter += 1

    def time(self):
        return self.controller.get_time() - self.time_start

    def __odometry_callback(self, message):
        orientation = message.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion((orientation.x,
                                                  orientation.y,
                                                  orientation.z,
                                                  orientation.w))
        self.gamma = pitch
