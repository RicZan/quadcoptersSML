#!/usr/bin/env python
# this line is just used to define the type of document

import rospy 

import numpy
from numpy import *
from numpy.linalg import *
from numpy import cos as c
from numpy import sin as s

from SomeFunctions import OP,skew,GetRotFromEulerAnglesDeg



class OscillTask():
    
    def __init__(self,parameters = None):
        if parameters != None:
            self.w  = parameters[0]
        else:
            self.w  = 0.1
        self.joint1 = 1000
        self.gripper = 1000
        self.t = 0


    def output(self,t):

        # frequency
        #w = 2
        w = self.w
        U = zeros(2)

        #self.t = self.t+1
        self.t = int(t*10)
        rospy.logwarn('self.t = ' + str(self.t))

        self.joint1 = (1 + sin(self.t*w))*90
        self.gripper = (1 + sin(self.t*w*2))*90

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
    U_PWM[1] = minPWM1 + (maxPWM1-minPWM1)*U[1]/180

    return U_PWM