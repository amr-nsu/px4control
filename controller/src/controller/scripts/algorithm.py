class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.5
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        BASE_THRUST = 0.45
        DELTA_Z_TO_THRUST = 0.2

        thrust = BASE_THRUST \
            + DELTA_Z_TO_THRUST * (self.z_ref - self.controller.get_position().z)
        roll, pitch, yaw = 0, 0, 0
        self.controller.set_control(roll, pitch, yaw, thrust)
