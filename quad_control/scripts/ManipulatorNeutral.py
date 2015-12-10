#!/usr/bin/env python
# this line is just used to define the type of document

import rospy 

import numpy
from numpy import *
from numpy.linalg import *
from numpy import cos as c
from numpy import sin as s

from SomeFunctions import OP,skew,GetRotFromEulerAnglesDeg



class NeutralTask():
    
    def __init__(self,parameters = None):
        self.joint1 = 1000
        self.gripper = 1000


    def output(self,t):

        U = zeros(2)

        self.joint1 = 0
        self.gripper = 0

        U[0] = self.gripper
        U[1] = self.joint1

        return ConvertToPWM(U)


def ConvertToPWM(U):
    
    U_PWM = zeros(2)

    minPWMg = 1000
    maxPWMg = 2400
    minPWM1 = 1000
    maxPWM1 = 2000

    U_PWM[0] = minPWMg + (maxPWMg-minPWMg)*U[0]/180
    U_PWM[1] = minPWM1 + (maxPWM1-minPWM1)*U[0]/180

    return U_PWM