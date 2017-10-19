class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        # 0.45; P: 0.2; D: 0
        z_ref = 0.5
        thrust = 0.45 \
            + 0.1 * (z_ref - self.controller.get_position().z) \
            - 0.01 * self.controller.get_linear_velocity().z

        y_ref, x_ref = 0, 0
        pitch = 0.1 * (x_ref - self.controller.get_position().x)
        roll = -0.1 * (y_ref - self.controller.get_position().y)
        yaw = 0

        self.controller.set_control(roll, pitch, yaw, thrust)
