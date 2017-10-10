import tf
import rospy

from coordinate import Coordinate
from tf.transformations import euler_from_quaternion


class CameraTracker:

    def __init__(self, marker_id='ar_marker_0'):

        self.listener = tf.TransformListener()
        self.marker_id = marker_id
        self.coordinate = Coordinate()
        rospy.Timer(rospy.Duration(0.01), lambda event: self.__timer_callback())

    def get_coordinate(self):
        return self.coordinate

    def __timer_callback(self):
        try:
            (p, q) = self.listener.lookupTransform('/world', self.marker_id, rospy.Time(0))
            euler = euler_from_quaternion(q)
            self.__update(Coordinate(p[0], p[1], p[2], euler[2]))
        except:
            pass

    def __update(self, coordinate):
        if -2 < coordinate.x < 2 and -2 < coordinate.y < 2 and -1 < coordinate.z < 3:
            self.coordinate.low_pass_filter(coordinate, T=0.1)

if __name__ == '__main__':
    rospy.init_node('camera_tracker')
    cam_tracker = CameraTracker()
    while not rospy.is_shutdown():
        print cam_tracker.get_coordinate()
        rospy.sleep(0.1)
