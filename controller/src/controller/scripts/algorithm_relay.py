class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):

        def relay(value, threshold):
            if -threshold <= value <= threshold:
                return 0
            return 1 if value > 0 else -1

        # 0.45; P: 0.2; D: 0
        z_ref = 0.5
        thrust = 0.45 \
            + 0.1 * (z_ref - self.controller.get_position().z) \
            - 0.01 * self.controller.get_linear_velocity().z

        x_ref, y_ref = 0, 0.2

        pitch = 0.05 * relay(x_ref - self.controller.get_position().x, 0.1)
        roll = -0.05 * relay(y_ref - self.controller.get_position().y, 0.1)
        yaw = 0

        self.controller.set_control(roll, pitch, yaw, thrust)
