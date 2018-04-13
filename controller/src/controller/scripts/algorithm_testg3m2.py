from geometry_msgs.msg import Point, Vector3
import math

def f(x, y):
	return x*x + y*y - 1

def z_ref(x, y):
	return 0,5

def v_ref(x, y):
	return 0,25

class DerivCalc:
	def __init__(self, initial_value):
		self.last_value = initial_value
		self.derivative = 0

	def update(self, value, delta):
		self.derivative = (value - self.last_value) / delta
		self.last_value = value

	def get(self):
		return self.derivative

class Algorithm:

	def __init__(self, controller):
		self.controller = controller
		self.df_dx = DerivCalc(0)
		self.df_dy = DerivCalc(0)
		self.df_dt = DerivCalc(0)
		self.d2f_dxx = DerivCalc(0)
		self.d2f_dyy = DerivCalc(0)
		self.d2f_dxy = DerivCalc(0)
	        self.z_ref = 0.5
		self.a_f = 1
		self.a_z = 1
		self.a_v = 1
		self.a_ang = Vector3()
		self.k_ang = Vector3()
		self.k_f = 1
		self.k_z = 1
		self.mass = 1.5
		self.inertia = Vector3(1, 1, 1)
		self.g = 9.81
		self.l = 0.25
		self.lamb = 1
		self.u = [0, 0, 0, 0]
		self.force = [0, 0, 0, 0]
	        self.controller.set_control_loop(0.01, self.loop)

	def loop(self):
		self.__update()
		self.__calculate_u()
		self.__calculate_forces()
		#self.controller.set_actuator_control([self.force[0], self.force[1], self.force[2], self.force[3], 0, 0, 0, 0])
		self.controller.set_actuator_control([1200, 1200, 1200, 1200, 0, 0, 0, 0])

	def __calculate_forces(self):
		l_inv = 1 / self.l
		half_lambda_inv = 0.5 / self.lamb
		self.force[0] = 0.5 * self.u[0] - l_inv * self.u[2] - half_lambda_inv * self.u[3]
		self.force[1] = -l_inv * self.u[1]
		self.force[2] =  l_inv * self.u[2]
		self.force[3] = 0.5 * self.u[0] + l_inv * self.u[1] + half_lambda_inv * self.u[3]


	def __calculate_u(self):
		S_f = self.df_dt.get() + self.k_f * self.f
		S_z = self.dzz_dt.get() + self.k_z * (self.position.z - self.z_ref)
		S_v = self.speed - self.v_ref
		A = -self.a_f * S_f - self.k_f * (self.df_dx.get() * self.l_velocity.x + self.df_dy.get() * self.l_velocity.y) - 2 * self.d2f_dxy.get() * self.l_velocity.x * self.l_velocity.y - self.d2f_dxx.get() * self.l_velocity.x * self.l_velocity.x - self.d2f_dyy.get() * self.l_velocity.y * self.l_velocity.y
		B = -self.a_v * S_v
		C = -self.a_z * S_z - self.k_z * self.l_velocity.z
		D = self.df_dx.get() * self.l_velocity.y - self.df_dx.get() * self.l_velocity.x
		D_1 = A * self.l_velocity.y - B * self.speed * self.df_dy.get()
		D_2 = B * self.speed * self.df_dx.get() - A * self.l_velocity.x
		DD = math.sqrt((D_1/D)**2 + (D_2/D)**2 + (self.g + C)**2)

		gamma = ( D_1/D/DD, D_2/D/DD, (self.g + C)/DD )
		ref_angles = self.__calculate_ref_angles(gamma)

		self.u[0] =  self.mass * DD
		self.u[1] =  self.inertia.x * ((self.a_ang.x + self.k_ang.x) * self.angular_velocity.x + self.a_ang.x * self.k_ang.x * (self.euler.x - ref_angles.x)) + (self.inertia.z - self.inertia.y) * self.angular_velocity.y * self.angular_velocity.z
		self.u[2] =  self.inertia.x * ((self.a_ang.y + self.k_ang.y) * self.angular_velocity.y + self.a_ang.y * self.k_ang.y * (self.euler.y - ref_angles.y)) + (self.inertia.x - self.inertia.z) * self.angular_velocity.x * self.angular_velocity.z
		self.u[3] = -self.inertia.x * ((self.a_ang.z + self.k_ang.z) * self.angular_velocity.z + self.a_ang.z * self.k_ang.z * (self.euler. - ref_angles.z))

	def __calculate_ref_angles(self, gamma):
		angles = Vector3()

		v_psi = math.hypot(self.df_dx.get(), self.df_dy.get())
		sin_psi = -(self.df_dx.get() / v_psi)
		cos_psi =   self.df_dy.get() / v_psi
		angles.z = math.atan2(sin_psi, cos_psi)

		v_phi = gamma[0] * sin_psi - gamma[1] * cos_psi
		sin_phi = v_phi
		cos_phi = math.sqrt(1 - (v_phi)**2)
		angles.x = math.atan2(sin_phi, cos_phi)

		v_nu_top = gamma[1] * sin_psi + gamma[0] * sin_psi
		v_nu_bottom = math.hypot(gamma[2], v_nu_top)
		sin_nu = v_nu_top / v_nu_bottom
		cos_nu = gamma[2] / v_nu_bottom
		angles.y = math.atan2(sin_nu, cos_nu)

		return angles


	def __update(self):
		time = self.controller.get_time()
		position = self.controller.get_vision_position()
		euler = self.controller.get_euler()
		l_velocity = self.controller.get_linear_velocity()

		time_delta = time - self.last_update_time
		pos_delta = position - self.position
		linear_vel_delta = l_velocity - self.linear_velocity

		self.last_update_time = time
		self.position = position
		self.euler = Vector3(euler[0], euler.y[1], euler[2])
		self.linear_velocity = l_velocity
		self.angular_velocity = self.controller.get_angular_velocity()

		self.speed = math.hypot(l_velocity.x, l_velocity.y)
		self.f = f(position.x, position.y)
		self.z_ref = z_ref(x, y)
		self.v_ref = v_ref(x, y)

		self.df_dx.update(f, pos_delta.x)
		self.df_dy.update(f, pos_delta.y)
		self.df_dt.update(f, time_delta)
		self.dzz_dt.update((position.z - self.z_ref), time_delta)

		self.d2f_dxx.update(self.df_dx.get(), pos_delta.x)
		self.d2f_dyy.update(self.df_dy.get(), pos_delta.y)
		self.d2f_dxy.update(self.df_dx.get(), pos_delta.y)


