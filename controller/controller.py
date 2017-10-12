import math
import rospy

from coordinate import Coordinate
from camera_tracker import CameraTracker
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ManualControl, Thrust
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def saturation(value, lover_value, upper_value):
    if value < lover_value:
        return lover_value
    elif value > upper_value:
        return upper_value
    return value


class Controller:

    def __init__(self):

        self.camera_tracker = CameraTracker()
        self.coordinate = Coordinate()
        self.manual_control = None

        self.setpoint_attitude_pub = rospy.Publisher('mavros/setpoint_attitude/target_attitude',
                                                     PoseStamped, queue_size=1)
        self.setpoint_thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust',
                                                     Thrust, queue_size=1)

        rospy.Timer(rospy.Duration(0.01), lambda event: self.__update_coordinate())

        rospy.Subscriber('mavros/manual_control/control', ManualControl,
                         self.__manual_control_callback)

    def get_coordinate(self):
        return self.coordinate

    def set_control_loop(self, period, callback):
        rospy.Timer(rospy.Duration(period), lambda event: callback())

    def set_control(self, roll, pitch, yaw, thrust):
        if self.manual_control is not None:
            CONTROL_TO_DEG = 0.1
            roll += self.manual_control.y * CONTROL_TO_DEG
            pitch += self.manual_control.x * CONTROL_TO_DEG
            yaw += -self.manual_control.r * math.pi
            thrust = saturation(thrust, 0, self.manual_control.z)
        self.__set_thrust(thrust)
        self.__set_attitude(roll, pitch, yaw)

    def __set_thrust(self, thrust):
        msg = Thrust()
        msg.header = Header()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = rospy.Time.now()
        msg.thrust = thrust
        self.setpoint_thrust_pub.publish(msg)
        rospy.loginfo('thrust(%.2f)' % (thrust))

    def __set_attitude(self, roll, pitch, yaw):
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = 'base_footprint'
        pos.header.stamp = rospy.Time.now()
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pos.pose.orientation = Quaternion(*quaternion)
        self.setpoint_attitude_pub.publish(pos)
        rospy.loginfo('attitude(%.2f, %.2f, %.2f)' % (roll, pitch, yaw))

    def __manual_control_callback(self, message):
        self.manual_control = message

    def __update_coordinate(self):
        self.coordinate = self.camera_tracker.get_coordinate()
