import rospy

import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

class VisionPose:

    def __init__(self):
        self.marker_id = 'ar_marker_0'

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
                    pose = PoseStamped()
                    pose.header = Header()
                    pose.header.stamp = transform.header.stamp
                    pose.pose.position.x = p_list[0]
                    pose.pose.position.y = p_list[1]
                    pose.pose.position.z = p_list[2]
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
