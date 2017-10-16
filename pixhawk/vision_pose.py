import rospy
from camera_tracker import CameraTracker
from geometry_msgs.msg import PoseStamped


class VisionPose:

    def __init__(self):
        self.camera_tracker = CameraTracker()
        rospy.Timer(rospy.Duration(0.1), lambda event: self.__timer_callback())
        self.vision_pose_estimate_pose_pub = rospy.Publisher('mavros/vision_pose/pose',
                                                             PoseStamped, queue_size=1)

    def __timer_callback(self):
        pose = self.camera_tracker.get_pose()
        self.vision_pose_estimate_pose_pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('vision_pose')
    v = VisionPose()
    rospy.spin()