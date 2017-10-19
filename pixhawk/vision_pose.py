import rospy

import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped


class Area:

    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

    def check(self, x, y, z):
        return self.x_min <= x <= self.x_max \
            and self.y_min <= y <= self.y_max \
            and self.z_min <= z <= self.z_max


class VisionPose:

    def __init__(self):
        self.marker_id = 'ar_marker_0'
        self.area = Area(-1.5, 1.5, -1.5, 1.5, -0.5, 2.0)
        self.transform_listener = tf.TransformListener()
        rospy.Subscriber("tf", TFMessage,
                         self.__tf_message_callback)

        self.vision_pose_estimate_pose_pub = rospy.Publisher('mavros/vision_pose/pose',
                                                             PoseStamped, queue_size=1)

    def __tf_message_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == self.marker_id:
                try:
                    (p_list, q_list) = self.transform_listener.lookupTransform('/world', self.marker_id, transform.header.stamp)
                    x, y, z = p_list[0], p_list[1], p_list[2]
                    if self.area.check(x, y, z):
                        pose = PoseStamped()
                        pose.header = Header()
                        pose.header.stamp = transform.header.stamp
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.position.z = z
                        pose.pose.orientation.x = q_list[0]
                        pose.pose.orientation.y = q_list[1]
                        pose.pose.orientation.z = q_list[2]
                        pose.pose.orientation.w = q_list[3]
                        self.vision_pose_estimate_pose_pub.publish(pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                    rospy.logwarn(e)


if __name__ == '__main__':
    rospy.init_node('vision_pose')
    v = VisionPose()
    rospy.spin()
