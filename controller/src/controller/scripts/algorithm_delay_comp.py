import rospy
from math import *

class Model:
    def __init__(self, z=0.):
        # init state
        self.z = z

        # init speed
        self.dz = 0.
        
   
    def step(self, u3, u1, dt):
        
        d2z = (u1 - (M1+M2)*G)/(M1+M2)
        self.dz += d2z * dt
        self.z += self.dz * dt 
class Algorithm:

     def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.8
        self.themodel = Model(z = 0)
        self.z = controller.get_position().z
        self.x = controller.get_position().x
        self.counter = 0
        self.time_start = self.controller.get_time()
        self.log_model = open('log/%s_regulation' % self.time_start, 'w')
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        roll, pitch, yaw = 0, 0, 0

        thrust_buf = []
        
        k_z = 1.5
        alpha = k_z
        G = 9.81
        n_delay = 50
        
        z_gr = self.controller.get_position().z
        vz = self.controller.get_linear_velocity().z       

        Az = self.themodel.dz * (k_z + alpha) + k_z * alpha * (self.themodel.z - self.z_ref) - G

        Azz = Az

        M1 = 0.42
        norm_coef = 1. * 0.45 / (M1 * G)
        
        thrust = norm_coef * (M1 * sqrt(Azz * Azz))      
        if (self.counter > n_delay):
        
          self.themodel.z = z_gr
          self.themodel.dz = vz
          
           for ii in range(0,n_delay):
                self.themodel.step(0, thrust_buf[self.counter + ii-n_delay],0.01)
        #   thrust = norm_coef*thrust_buf[self.counter - n_delay]
        #thrust = norm_coef * (M1 * sqrt(Azz * Azz + Axx * Axx)

        rospy.loginfo('thrust(%.2f) z %.2f vz %.2f x %.2f vx %.2f' % (thrust, z_gr, vz, x_gr, vx))
        self.log_model.write("%f %f %f %f \n" % (self.time(), thrust, z_gr, vz)
        self.log_model.flush()

        self.controller.set_control(roll, pitch, yaw, thrust)
        self.counter += 1
    def time(self):
        return self.controller.get_time() - self.time_start
