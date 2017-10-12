import rospy
from algorithm import Algorithm
from controller import Controller

if __name__ == '__main__':
    rospy.init_node('px4control')
    controller = Controller()
    algorithm = Algorithm(controller)
    rospy.spin()