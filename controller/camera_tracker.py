import tf
import rospy

from coordinate import Coordinate
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped

class CameraTracker:

    def __init__(self, marker_id='ar_marker_0'):

        self.listener = tf.TransformListener()
        self.marker_id = marker_id
        self.coordinate = Coordinate()
        self.pose = PoseStamped()
        rospy.Timer(rospy.Duration(0.1), lambda event: self.__timer_callback())

    def get_coordinate(self):
        return self.coordinate

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
            self.__update(Coordinate(p_list[0], p_list[1], p_list[2], yaw))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def __update(self, coordinate):
        if -2 < coordinate.x < 2 and -2 < coordinate.y < 2 and -1 < coordinate.z < 3:
            self.coordinate.low_pass_filter(coordinate, T=0.1)


if __name__ == '__main__':
    rospy.init_node('camera_tracker')
    cam_tracker = CameraTracker()
    while not rospy.is_shutdown():
#        print cam_tracker.get_coordinate()
        rospy.sleep(0.1)
