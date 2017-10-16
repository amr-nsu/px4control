import math
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Quaternion, Vector3
from mavros_msgs.msg import ManualControl, Thrust
from tf.transformations import quaternion_from_euler

def saturation(value, lover_value, upper_value):
    if value < lover_value:
        return lover_value
    elif value > upper_value:
        return upper_value
    return value


class Controller:

    def __init__(self):

        self.setpoint_attitude_pub = rospy.Publisher('mavros/setpoint_attitude/target_attitude',
                                                     PoseStamped, queue_size=1)
        self.setpoint_thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust',
                                                   Thrust, queue_size=1)

        self.position = Point()
        self.orientation = Quaternion()
        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                 self.__local_position_pose_callback)

        self.linear_velocity = Vector3()
        self.angular_velocity = Vector3()
        rospy.Subscriber("mavros/local_position/velocity", TwistStamped,
                 self.__local_position_velocity_callback)

        self.manual_control = None
        rospy.Subscriber('mavros/manual_control/control', ManualControl,
                         self.__manual_control_callback)

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation

    def get_linear_velocity(self):
        return self.linear_velocity

    def get_angular_velocity(self):
        return self.angular_velocity

    def get_time(self):
        return rospy.Time.now().to_sec()

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

    def __local_position_pose_callback(self, message):
        self.position = message.pose.position
        self.orientation = message.pose.orientation

    def __local_position_velocity_callback(self, message):
        self.linear_velocity = message.twist.linear
        self.angular_velocity = message.twist.angular

    def __manual_control_callback(self, message):
        self.manual_control = message
