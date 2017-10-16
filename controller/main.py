import rospy
from algorithm_studSasha import Algorithm
#from algorithm_01 import Algorithm
from controller import Controller

if __name__ == '__main__':
    rospy.init_node('px4control')
    controller = Controller()
    algorithm = Algorithm(controller)
    rospy.spin()
