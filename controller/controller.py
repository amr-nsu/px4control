import math
import rospy

from std_msgs.msg import Header
from coordinate import Coordinate
from geometry_msgs.msg import PoseStamped, Quaternion
from camera_tracker import CameraTracker
from attitude import Attitude
from mavros_msgs.msg import ManualControl, Thrust
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def saturation(value, lover_value, upper_value):
    if value < lover_value:
        return lover_value
    elif value > upper_value:
        return upper_value
    return value

class Controller:

    CONTROL_TO_DEG = 0.1
    CONTROL_TO_THRUST = 0.2
    CONTROL_THROTTLE_BASE = 0.45

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
        rospy.Timer(rospy.Duration(0.1), lambda event: self.control())

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
            yaw += -self.manual_control.r

        self.set_attitude(roll, pitch, yaw)
        self.set_throttle()

    def set_throttle(self):
        if self.manual_control is None :
            return

        z_ref = 0.5
        delta = z_ref - self.coordinate.z
        thr = Controller.CONTROL_THROTTLE_BASE + Controller.CONTROL_TO_THRUST * delta

        msg = Thrust()
        msg.header = Header()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = rospy.Time.now()
        msg.thrust = saturation(thr, 0, self.manual_control.z)
        # rospy.loginfo('throttle(%.2f, %.2f, %.2f, %.2f)'
        #               % (z_ref, delta, thr, msg.thrust))
        self.setpoint_thrust_pub.publish(msg)

    def set_attitude(self, roll, pitch, yaw):
        def wrap_to_pi(angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle

        rospy.loginfo('attitude(%.2f, %.2f, %.2f)' % (roll, pitch, yaw))
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = 'base_footprint'
        pos.header.stamp = rospy.Time.now()

        yaw_fixed = wrap_to_pi(yaw - self.attitude.yaw_fix)
        print yaw_fixed, self.attitude.yaw_fix

        quaternion = quaternion_from_euler(roll, -pitch, yaw_fixed)
        pos.pose.orientation = Quaternion(*quaternion)
        self.setpoint_attitude_pub.publish(pos)

if __name__ == '__main__':
    rospy.init_node('controller')
    contoller = Controller()
    while not rospy.is_shutdown():
        pass



