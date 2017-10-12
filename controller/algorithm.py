BASE_THRUST = 0.45
CONTROL_TO_THRUST = 0.2

class Algorithm:

    def __init__(self, controller):
        self.controller = controller
        self.z_ref = 0.5
        self.controller.set_control_loop(0.01, self.loop)

    def loop(self):
        coordinate = self.controller.get_coordinate()

        delta = self.z_ref - coordinate.z
        thrust = BASE_THRUST + CONTROL_TO_THRUST * delta

        roll, pitch, yaw = 0, 0, 0

        self.controller.set_control(roll, pitch, yaw, thrust)
