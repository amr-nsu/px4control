#!/usr/bin/env python2

import math
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ManualControl, Thrust
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from camera_tracker import CameraTracker


def saturation(value, lover_value, upper_value):
    if value < lover_value:
        return lover_value
    elif value > upper_value:
        return upper_value
    return value

class Pose:
    def __init__(self, x=0, y=0, r=0):
        self.x = x
        self.y = y
        self.r = r

class Controller:

    CONTROL_TO_DEG = 0.1
    CONTROL_TO_THRUST = 0.2
    CONTROL_THROTTLE_BASE = 0.45
    DELTA_TO_YAW = 1.0 / 2 / math.pi

    def __init__(self):
        self.start_time = rospy.get_time()
        self.local_position = None

        self.ground_level = None
        self.yaw_level = None
        self.z_ref = 0
        self.pose_ref = Pose(0, 0, 0)

        self.manual_control = None
        self.pose_markers = None
        self.marker_id = 0

        self.camera_tracker = CameraTracker()

        rospy.Subscriber("mavros/local_position/pose", PoseStamped,
                         self.local_position_pose_callback)
        rospy.Timer(rospy.Duration(0.05), lambda event: self.step())
        self.setpoint_attitude_pub = rospy.Publisher('mavros/setpoint_attitude/target_attitude',
                                                     PoseStamped, queue_size=1)
        self.setpoint_thrust_pub = rospy.Publisher('mavros/setpoint_attitude/thrust',
                                                     Thrust, queue_size=1)
        rospy.Subscriber('mavros/manual_control/control', ManualControl,
                         self.manual_control_callback)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers,
                         self.pose_marker_callback)


    def manual_control_callback(self, message):
        self.manual_control = message

    def attitude(self, roll, pitch, yaw):
        rospy.loginfo('attitude(%.2f, %.2f, %.2f)' % (roll, pitch, yaw))
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
        if self.local_position is None or self.manual_control is None :
            return

        delta = self.z_ref - (self.local_position.pose.position.z - self.ground_level)
        thr = Controller.CONTROL_THROTTLE_BASE + Controller.CONTROL_TO_THRUST * delta
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
#        print self.camera_tracker.get_coordinate()
        return

        current_time = rospy.get_time() - self.start_time
        self.z_ref = 0.5

        if self.manual_control is not None:
            self.attitude(self.manual_control.y * Controller.CONTROL_TO_DEG,
                          self.manual_control.x * Controller.CONTROL_TO_DEG,
                          -self.manual_control.r)
            # control = self.get_control()
            # if control is not None:
            #     ctr = control
            # else:
            #     ctr = self.manual_control
            # self.attitude(self.manual_control.y * Controller.CONTROL_TO_DEG,
            #               self.manual_control.x * Controller.CONTROL_TO_DEG,
            #               -ctr.r)
        self.throttle()

    def get_control(self):
        pose = self.get_marker_pose()
        if pose is not None:
            euler = euler_from_quaternion([pose.orientation.w,
                                           pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z])


            yaw = euler[0]
            rospy.loginfo('get_control(%.2f, %.2f, %.2f)'
                    % (pose.position.x, pose.position.y, yaw))
            delta_x = self.pose_ref.x - pose.position.x
            delta_y = self.pose_ref.y - pose.position.y

            if self.yaw_level is not None:
                delta_r = self.yaw_level * DELTA_TO_YAW
            else:
                delta_r = 0

            control = Pose()
            control.x = saturation(1.0 * delta_x, -1, 1)
            control.y = saturation(1.0 * delta_y, -1, 1)
            control.r = saturation(1.0 * delta_r, -1, 1)
            return control
        return None

    def get_marker_pose(self):
        if self.pose_markers is not None:
            for marker in self.pose_markers.markers:
                if marker.id == self.marker_id:
                    return marker.pose.pose
        return None

    def local_position_pose_callback(self, message):
        print euler_from_quaternion([message.pose.orientation.w,
                                           message.pose.orientation.x,
                                           message.pose.orientation.y,
                                           message.pose.orientation.z])
        return
        if self.manual_control is not None:
            print self.manual_control.r

        self.local_position = message
        if self.ground_level is None:
            self.ground_level = message.pose.position.z
        if self.yaw_level is None:
            euler = euler_from_quaternion([message.pose.orientation.w,
                                           message.pose.orientation.x,
                                           message.pose.orientation.y,
                                           message.pose.orientation.z])
            self.yaw_level = euler[0]

    def pose_marker_callback(self, message):
        self.pose_markers = message

if __name__ == '__main__':
    rospy.init_node('controller')
    controller = Controller()
    rospy.spin()
