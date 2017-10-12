import math
import rospy

from attitude import Attitude
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

    BASE_THRUST = 0.45
    CONTROL_TO_DEG = 0.1
    CONTROL_TO_THRUST = 0.2
    DELTA_ANGLE = 2.0

    def __init__(self):

        self.camera_tracker = CameraTracker()
        self.attitude = Attitude()
        self.manual_control = None
        self.coordinate = Coordinate()

        self.setpoint_attitude_pub = rospy.Publisher('mavros/setpoint_attitude/target_attitude',
                                                     PoseStamped, queue_size=1)
        self.setpoint_thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust',
                                                     Thrust, queue_size=1)

        rospy.Timer(rospy.Duration(0.01), lambda event: self.update())
        rospy.Timer(rospy.Duration(0.02), lambda event: self.control())

        rospy.Subscriber('mavros/manual_control/control', ManualControl,
                         self.manual_control_callback)

    def manual_control_callback(self, message):
        self.manual_control = message

    def update(self):
        self.coordinate = self.camera_tracker.get_coordinate()
        self.attitude.update(self.coordinate.yaw)

    def control(self):
        roll, pitch, yaw = 0, 0, 0
        # yaw = math.pi/10 * math.sin(0.1 * rospy.get_time())

        if self.manual_control is not None:
            roll += self.manual_control.y * Controller.CONTROL_TO_DEG
            pitch += -self.manual_control.x * Controller.CONTROL_TO_DEG
            yaw += -self.manual_control.r * math.pi

        self.set_attitude(roll, pitch, yaw)
        self.set_thrust(z_ref=0.5)

    def set_thrust(self, z_ref):
        delta = z_ref - self.coordinate.z
        thrust = Controller.BASE_THRUST + Controller.CONTROL_TO_THRUST * delta

        if self.manual_control is not None:
            thrust = saturation(thrust, 0, self.manual_control.z)

        msg = Thrust()
        msg.header = Header()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = rospy.Time.now()
        msg.thrust = thrust
        self.setpoint_thrust_pub.publish(msg)

        rospy.loginfo('thrust(%.2f, %.2f)' % (delta, msg.thrust))

    def set_attitude(self, roll, pitch, yaw):
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = 'base_footprint'
        pos.header.stamp = rospy.Time.now()

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pos.pose.orientation = Quaternion(*quaternion)
        self.setpoint_attitude_pub.publish(pos)

        rospy.loginfo('attitude(%.2f, %.2f, %.2f)' % (roll, pitch, yaw))


if __name__ == '__main__':
    rospy.init_node('controller')
    contoller = Controller()
    rospy.spin()
