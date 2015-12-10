#!/usr/bin/env python

import rospy
import numpy
from numpy import *
from numpy import cos as c
from numpy import sin as s
from std_msgs.msg import Float32

from quad_control.msg import ManipulatorMsg

class SetManipulator():

    def __init__(self):
        self.startGripper = 1000
        self.endGripper = 1000
        self.startJoint = 1000
        self.endJoint = 1000
        self.ct = 0
    

    def ConvertToPWM(U):

    U_PWM = zeros(2)

    minPWMg = 1000
    maxPWMg = 2400
    minPWM1 = 1000
    maxPWM1 = 2000

    U_PWM[0] = minPWMg + (maxPWMg-minPWMg)*U[0]/180
    U_PWM[1] = minPWM1 + (maxPWM1-minPWM1)*U[1]/180

    return U_PWM


    def task_compute(self):
        rospy.init_node('manip_control', anonymous=True)

        manipPublisher = rospy.Publisher('manipulator_pose', ManipulatorPose, queue_size=10)
        freq = rospy.Rate(10)


        # x = numpy.linspace(-numpy.pi, numpy.pi, 21)
        t = 0

        while not rospy.is_shutdown():
            
            pose = ManipulatorPose()
            j1 = self.startJoint
            j2 = self.endJoint
            g1 = self.startGripper
            g2 = self.endGripper


            # WORK IN PROGRESS....
            # if p == y:
            #     pass
            # else:
            #     pose.Joint1 = j1 + 1
            #     pose.Gripper = (1 + sin(x))*90

            manipPublisher.publish(pose)
            freq.sleep()

        rospy.spin()


if __name__ == '__main__':
    try:
        manipulator = SetManipulator()
        manipulator.task_compute()
    except rospy.ROSInterruptException:
        pass