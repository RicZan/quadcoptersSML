#!/usr/bin/env python
# this line is just used to define the type of document

import numpy
from numpy import *
from numpy.linalg import *

class traj_des_trapez_vertical():

    def __init__(self,offset,Rotation,parameters=None):

        self.offset   = offset

        self.Rotation = Rotation

        # time
        self.tc = parameters[0]
        self.tf = parameters[1]

        # acc
        self.ad = parameters[2]

        # position
        self.zi = parameters[3]
        self.zf = parameters[4]


    # trajectory 
    def traj_des(self,t):

        ad = self.ad

        tc = self.tc
        tf = self.tf

        zi = self.zi
        zf = self.zf
        
        if t<=tc:
            p = array([ 0,0, zi - 0.5*ad*t**2])
            v = array([ 0,0, -ad*t])
            a = array([ 0,0, -ad])
            j = array([ 0,0, 0])
            s = array([ 0,0, 0])
        elif t<= tf-tc:
            p = array([ 0,0, zi - ad*tc*(t-tc/2)])
            v = array([ 0,0, -ad*tc])
            a = array([ 0,0, 0])
            j = array([ 0,0, 0])
            s = array([ 0,0, 0])
        elif t<= tf:
            p = array([ 0,0, zf + 0.5*ad*(tf-t)**2])
            v = array([ 0,0, -ad*(tf-t)])
            a = array([ 0,0, ad])
            j = array([ 0,0, 0])
            s = array([ 0,0, 0])
        else:
            p = array([ 0,0, zf])
            v = array([ 0,0, 0])
            a = array([ 0,0, 0])
            j = array([ 0,0, 0])
            s = array([ 0,0, 0])

        
        return numpy.concatenate([p,v,a,j,s])

    def output(self,t):
        return self.add_offset_and_rot(self.traj_des(t))


    def add_offset_and_rot(self,position):

        pos_out = numpy.zeros(3*5)
        
        R = self.Rotation
        
        pos_out[0:3]   = R.dot(position[0:3])
        pos_out[3:6]   = R.dot(position[3:6])
        pos_out[6:9]   = R.dot(position[6:9])
        pos_out[9:12]  = R.dot(position[9:12])
        pos_out[12:15] = R.dot(position[12:15])

        pos_out[0:3] = pos_out[0:3] + self.offset   

        return pos_out