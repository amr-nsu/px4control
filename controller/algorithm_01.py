class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.5
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        # 0.45; P: 0.2; D: 0
        thrust = 0.45 \
            + 0.1 * (self.z_ref - self.controller.get_position().z) \
            - 0.01 * (self.controller.get_linear_velocity().z)
        roll, pitch, yaw = 0, 0, 0
        self.controller.set_control(roll, pitch, yaw, thrust)
