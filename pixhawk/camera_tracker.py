import tf
import rospy

from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped

class CameraTracker:

    def __init__(self, marker_id='ar_marker_0'):

        self.listener = tf.TransformListener()
        self.marker_id = marker_id
        self.pose = PoseStamped()
        rospy.Timer(rospy.Duration(0.1), lambda event: self.__timer_callback())

    def get_pose(self):
        return self.pose

    def __timer_callback(self):
        try:
            (p_list, q_list) = self.listener.lookupTransform('/world', self.marker_id, rospy.Time(0))
            self.pose.header = Header()
            self.pose.header.stamp = rospy.Time.now()
            self.pose.pose.position.x = p_list[0]
            self.pose.pose.position.y = p_list[1]
            self.pose.pose.position.z = p_list[2]
            self.pose.pose.orientation.x = q_list[0]
            self.pose.pose.orientation.y = q_list[1]
            self.pose.pose.orientation.z = q_list[2]
            self.pose.pose.orientation.w = q_list[3]
            (roll, pitch, yaw) = euler_from_quaternion(q_list)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    rospy.init_node('camera_tracker')
    camera_tracker = CameraTracker()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
