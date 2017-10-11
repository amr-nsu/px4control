import rospy
from math import sqrt


class Coordinate:

    def __init__(self, x=0, y=0, z=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vyaw = 0

        self.timestamp = rospy.get_time()

    def __repr__(self):
        return 'Coordinate\n\t'\
               'x: %f\n\ty: %f\n\tz: %f\n\tyaw: %f\n\ttimestamp: %f'\
                % (self.x, self.y, self.z, self.yaw, self.timestamp)

    def get_distance(self, coordinate):
        return sqrt((coordinate.x - self.x)**2 +
                    (coordinate.y - self.y)**2 +
                    (coordinate.z - self.z)**2)

    def low_pass_filter(self, coordinate, T):
        """lpf with transfer function W(s) = 1 / (Ts + 1)"""

        dt = coordinate.timestamp - self.timestamp

        self.vx = (coordinate.x - self.x) / T
        self.vy = (coordinate.y - self.y) / T
        self.vz = (coordinate.z - self.z) / T
        self.vyaw = (coordinate.yaw - self.yaw) / T

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        self.yaw += self.vyaw * dt

        self.timestamp = coordinate.timestamp