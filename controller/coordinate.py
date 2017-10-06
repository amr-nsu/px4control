import rospy
from math import sqrt

class Coordinate:

    def __init__(self, x=0, y=0, z=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
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
        self.x +=   1. / T * (coordinate.x - self.x) * dt
        self.y +=   1. / T * (coordinate.y - self.y) * dt
        self.z +=   1. / T * (coordinate.z - self.z) * dt
        self.yaw += 1. / T * (coordinate.yaw - self.yaw) * dt
        self.timestamp = coordinate.timestamp