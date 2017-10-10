import rospy

from coordinate import Coordinate
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class Attitude:

    def __init__(self):

        self.yaw = 0
        self.yaw_delta = 0
        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                 self.__local_position_pose_callback)

    def get_yaw(self):
        return self.yaw + self.yaw_delta

    def update(self, yaw):
        self.yaw_delta = yaw - self.yaw

    def __local_position_pose_callback(self, message):
        euler = euler_from_quaternion([message.pose.orientation.w,
                                       message.pose.orientation.x,
                                       message.pose.orientation.y,
                                       message.pose.orientation.z])
        self.yaw = -euler[0]


if __name__ == '__main__':
    rospy.init_node('attitude')
    attitude = Attitude()
    rospy.spin()
