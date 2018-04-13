class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.5
	self.a_z = 1
	self.k_z = 1
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
	z = self.controller.get_position().z;
	dz = self.controller.get_linear_velocity().z;
#	thrust = -(self.a_z + self.k_z) * dz - self.a_z * self.k_z * (z - self.z_ref)
	thrust = 0.2
        roll, pitch, yaw = 0, 0, 0
#	self.controller.set_control(roll, pitch, yaw, thrust)
	self.controller.set_actuator_control([0.2, 0, 0.2, 0, 0, 0, 0, 0])
