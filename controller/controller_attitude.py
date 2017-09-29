#!/usr/bin/env python2

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from std_msgs.msg import Float64
from mavros_msgs.msg import ManualControl, Thrust
from tf.transformations import quaternion_from_euler

class Controller:

    CONTROL_TO_DEG = 0.1
    CONTROL_THROTTLE_BASE = 0.45

    def __init__(self):
        self.start_time = rospy.get_time()
        self.local_position = None
        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                         self.local_position_pose_callback)
        rospy.Timer(rospy.Duration(0.05), lambda event: self.step())
        self.setpoint_attitude_pub = rospy.Publisher('mavros/setpoint_attitude/target_attitude',
                                                     PoseStamped, queue_size=1)
        self.setpoint_thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust',
                                                     Thrust, queue_size=1)
        self.setpoint_position_local_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('mavros/manual_control/control', ManualControl,
                         self.manual_control_callback)

        self.throttle_gain = 0.2
        self.ground_level = None
        self.z_ref = 0

        self.manual_control = None


    def manual_control_callback(self, message):
        self.manual_control = message

    def attitude(self, roll, pitch, yaw):
        rospy.loginfo('attitude(%s, %s, %s)' % (roll, pitch, yaw))
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = 'base_footprint'
        pos.header.stamp = rospy.Time.now()
        pos.pose.position.x = 0
        pos.pose.position.y = 0
        pos.pose.position.z = 0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pos.pose.orientation = Quaternion(*quaternion)
        self.setpoint_attitude_pub.publish(pos)

    def throttle(self):
        def saturation(value, lover_value, upper_value):
            if value < lover_value:
                return lover_value
            elif value > upper_value:
                return upper_value
            return value

        if self.local_position is None or self.manual_control is None :
            return

        delta = self.z_ref - (self.local_position.pose.position.z - self.ground_level)
        thr = Controller.CONTROL_THROTTLE_BASE + self.throttle_gain * delta
        thr_manual = self.manual_control.z

        msg = Thrust()
        msg.header = Header()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = rospy.Time.now()
        msg.thrust = saturation(thr, 0, thr_manual)
        rospy.loginfo('throttle(%.2f, %.2f, %.2f, %.2f)'
                      % (self.z_ref, delta, thr, msg.thrust))
        self.setpoint_thrust_pub.publish(msg)

    def step(self):
        current_time = rospy.get_time() - self.start_time
        self.z_ref = 0.5
        if self.manual_control is not None:
            self.attitude(self.manual_control.y * Controller.CONTROL_TO_DEG,
                          self.manual_control.x * Controller.CONTROL_TO_DEG,
                          -self.manual_control.r)
            self.throttle()

    def local_position_pose_callback(self, message):
        self.local_position = message
        if self.ground_level is None:
            self.ground_level = message.pose.position.z

if __name__ == '__main__':
    rospy.init_node('controller')
    controller = Controller()
    rospy.spin()
