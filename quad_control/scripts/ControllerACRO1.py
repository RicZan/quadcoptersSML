#!/usr/bin/env python
# this line is just used to define the type of document

import rospy 

import sys

import numpy
from numpy import *
from numpy.linalg import *
from numpy import cos as c
from numpy import sin as s

from SomeFunctions import OP,skew,GetRotFromEulerAnglesDeg
from SomeFunctions import GetRotFromEulerAnglesDeg,Velocity_Filter

from Controllers_Parameters import parameters_sys


e3 = array([0, 0, 1])   # third canonical basis vector
g = 9.81                # gravity acceleration

class ControllerThrustOmega():

    # DEFAULT PARAMETERS (setted in "Controllers_Parameters.py": quad mass, gravity, ecc...)
    parameters = parameters_sys()   

    def __init__(self,parameters = None):

        if parameters != None:          
            # Parameters setted when the object "Controller" is created         
            # for example inside "cycle_quad_control_mavros.py" are setted from the GUI
            # with a service wich use the callback function "handle_controller_select"      
            self.parameters.ks   = parameters[0]
            self.parameters.katt  = parameters[1]
            self.parameters.kb  = parameters[2]
            self.parameters.ky  = parameters[3]
            self.parameters.kix  = parameters[4]
            self.parameters.kiy  = parameters[5]
            self.parameters.kiz  = parameters[6]
        
        self.timeNew  = parameters[7]
        self.timeOld  = self.timeNew

        self.SwitchIntegral  = parameters[8]
        self.dEstimateCurrent = 0

        self.SwitchManipulator  = parameters[9]
        #self.loadMass = parameters[10]

        self.RotationMatrixDerivative = MatrixDerivative(3,zeros((3,3)),0)

    def load(self,massLoad):
        self.parameters.massLoad = massLoad


    def output(self,t,states,states_d):

        self.timeNew = t

        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = GetRotFromEulerAnglesDeg(ee)
        R  = reshape(R,9)

        # collecting states
        states  = concatenate([states[0:6],R])
        return self.controller(states,states_d)


    ##################################################################################################
    ########################### CONTROLLER ###########################################################
    ##################################################################################################

    def controller(self,states,states_d):
        
        intON = self.SwitchIntegral

        # ****************** PARAMETERS ************************#

        self.refreshModel(states,states_d)

        # mass (kg)
        m = self.parameters.m 
        
        # quad position and velocity
        p  = states[0:3] 
        v  = states[3:6]

        # normal unit vector
        R  = states[6:15]           # v{I}=R*v{Q}
        R  = reshape(R,(3,3))
        n  = R.dot(e3)

        # desired quad trajectory
        pd = states_d[0:3]
        vd = states_d[3:6]
        ad = states_d[6:9]
        jd = states_d[9:12]
        sd = states_d[12:15]

        # position error and velocity error
        ep = p - pd
        ev = v - vd

        # ****************** CONTROL INPUTS ************************#
        
        # ---------------------- Thrust --------------------------#
        u = ad + g*e3 + self.uBounded(ep, ev) - self.dEstimate(ep, ev)*intON

        #throttle
        T = dot(u,n)    # T = u'*n     

        #thrust
        Thrust = m*T


        # ------------------ Angular Velocity ---------------------#
        
        # acceleration error
        ea = T*n - g*e3 - ad

        # derivative of "u"
        uDot = jd + self.uBoundedDot(ep, ev, ea) - self.dEstimateDot(ep, ev)*intON

        # desired attitude
        nd = u/norm(u)                                       

        # desired angular velocity 
        RtrSnd = dot(transpose(R),skew(nd))                         # R'*skew(nd)                      
        wd = dot(RtrSnd,uDot)/norm(u)                               # wd = (R'*skew(nd)*uDot)/norm(u)

        # partial derivative lyapunov function 
        dV2 = dV2_dev(ep, ev)

        # angular velocity    
        ks = self.parameters.ks
        katt = self.parameters.katt
        RtrSn = dot(transpose(R),skew(n)) 
        w = wd - ks*norm(u)*dot(RtrSn,dV2) - katt*dot(RtrSnd,n)  # w = wd-ks*norm(u)*(R'*skew(n)*dV2)-katt*(R'*skew(nd)*n)


        # ------------------ "Yaw" Control ---------------------#
        
        ky = self.parameters.ky       # gain
        psid = 0;                   # desired angle
        psidDot = 0;                # desired angular-velocity
        
        # current euler angles
        euler = GetEulerAngles(R);
        phi   = euler[0]
        theta = euler[1]
        psi   = euler[2]

        ephi = sin(psi-psid)    # angular error

        psiDot = psidDot - ky*ephi
        w[2] =(cos(theta)*psiDot-sin(phi)*w[1])/cos(phi)

        # ------------------ Control Inputs Vector ------------------#

        U = zeros(4)
        
        U[0] = Thrust
        U[1:4] = w

        U = Cmd_Converter(U, self.parameters)

        return U


    ################################# BOUNDED INPUT ####################################################


    # ------------------------------ Vectorial --------------------------------------------------------#

    def uBounded(self, ep, ev):
        out   = zeros(3)
        out[0] = self.uBounded_Scalar(ep[0], ev[0])
        out[1] = self.uBounded_Scalar(ep[1], ev[1])
        out[2] = self.uBounded_Scalar(ep[2], ev[2])
        return out

    def uBoundedDot(self, ep, ev, ea):
        out  = zeros(3)
        out[0] = self.uBoundedDot_Scalar(ep[0], ev[0], ev[0], ea[0])
        out[1] = self.uBoundedDot_Scalar(ep[1], ev[1], ev[1], ea[1])
        out[2] = self.uBoundedDot_Scalar(ep[2], ev[2], ev[2], ea[2])
        return out


    # ------------------------------ Scalar --------------------------------------------------------#

    def uBounded_Scalar(self, p, v):

        kb = self.parameters.kb 
        #rospy.loginfo("dOmega=%s, Omega+sigma=%s" % (dOmega(v), Omega(v)+sigma(p)))

        if abs(Omega(v)+sigma(p))<sys.float_info.epsilon:
            illFrac = 1
        else:
            illFrac = (v+sigma(p))/(Omega(v)+sigma(p)) 

        return - kb*(sigma(p)/dOmega(v))*illFrac - v*dsigma(p)/dOmega(v) - rho(Omega(v)+sigma(p))


    def uBoundedDot_Scalar(self, p, v, pDot, vDot):

        kb = self.parameters.kb

        Om = Omega(v)
        dOm = dOmega(v)
        ddOm = ddOmega(v)

        s = sigma(p)
        ds = dsigma(p)
        dds = ddsigma(p)

        dr = drho(Om+s)

        eps = sys.float_info.epsilon

        if abs(Om+s)<eps:
            illFrac1 = 1
            illFrac2 = 0
            illFrac3 = 1
        else:
            num1 = (s**2 + 2*s*Om + v*Om)
            den1 = (Om+s)**2
            illFrac1 = num1/den1

            num2 = (v+s)*dOm - (Om+s)
            den2 = (Om+s)**2
            illFrac2 = num2/den2

            illFrac3 = (v+s)/(Om+s)

        DubDp = -dr*ds - v*dds/dOm - kb*(ds/dOm)*illFrac1
        DubDv =  -ds/dOm - dr*dOm + v*ds*ddOm/(dOm**2) + kb*s*(illFrac2 + illFrac3*ddOm/(dOm**2))

        return DubDp*pDot + DubDv*vDot



    ################################ INTEGRAL ACTION #################################################

    # ------------------------------ Vectorial --------------------------------------------------------#

    def dEstimate(self, ep, ev):
        self.dEstimateCurrent = self.dEstimateCurrent + self.dEstimateDot(ep,ev)*(self.timeNew - self.timeOld)
        self.timeOld = self.timeNew
        return self.dEstimateCurrent

    def dEstimateDot(self, ep, ev):
        out = zeros(3)

        kix = self.parameters.kix
        kiy = self.parameters.kiy
        kiz = self.parameters.kiz

        out[0] = kix*self.dEstimateDot_Scalar(ep[0], ev[0])     #x
        out[1] = kiy*self.dEstimateDot_Scalar(ep[1], ev[1])     #y
        out[2] = kiz*self.dEstimateDot_Scalar(ep[2], ev[2])     #z

        return out

    # ------------------------------ Scalar --------------------------------------------------------#

    def dEstimateDot_Scalar(self, p, v):
        return (Omega(v)+sigma(p))*dOmega(v)



    ################################ MODEL WITH MANIPULATOR ###########################################

    def refreshModel(self,states,states_d):

        manipON = self.SwitchManipulator
        
        # ------------------------ Quad  -----------------------

        # quad mass 
        massQuad = self.parameters.massQuad                 # 1.442 Kg 

        # quad position and velocity
        posQuad  = states[0:3] 
        velQuad  = states[3:6]

        # quad rotation matrix
        R  = states[6:15]           # v{I}=R*v{Q}
        R  = reshape(R,(3,3))

        # ----------------- Manipulator  -----------------------

        # manipulator mass
        massManip = self.parameters.massManip               # 0.300 Kg 

        # manipulator sizes
        BODY_JOINT = 0.10   # 10cm
        JOINT_CM = 0.05     # 5cm
        CM_EEF = 0.12       # 12cm

        # joint1 angle
        gamma = states_d[15:16]        

        # manipulator position
        posManipFromBody = BODY_JOINT*array([0, 0, 1]) + JOINT_CM*array([cos(gamma), 0.005, sin(gamma)])
        posManip = posQuad - R.dot(posManipFromBody)

        # manipulator velocity
        RDot = self.RotationMatrixDerivative.output(self.timeNew, R)
        velManip = velQuad - RDot.dot(posManipFromBody)

        # ----------------- Load -----------------------

        massLoad = self.parameters.massLoad
        massManip = massManip #+ massLoad                                # 0.100 Kg (?) 
        posManip = posManip # ...

        # ----------------- Global System-----------------------
     
        # global mass
        m = massQuad + massManip*manipON 

        # global position
        p = posQuad - (massManip/m)*R.dot(posManipFromBody)*manipON     # = (posQuad*massQuad + posManip*massManip*manipON)/m 
        
        # global velocity
        v = velQuad - (massManip/m)*RDot.dot(posManipFromBody)*manipON             # we don't know gammaDot, so we assume to keep 
                                                                                    # the manipulator blocked...May God be with us!

        # ----------------- REFRESH -----------------------    

        self.parameters.m = m
        states[0:3] = p
        states[3:6] = v
        

##################################################################################################
########################### LYAPUNOV DERIVATIVES #################################################
##################################################################################################

# ------------------------------ Vectorial --------------------------------------------------------#

def dV2_dev(ep, ev):
    out = zeros(3)
    out[0] = dVb_dv(ep[0], ev[0])
    out[1] = dVb_dv(ep[1], ev[1])
    out[2] = dVb_dv(ep[2], ev[2])
    return out

# ------------------------------ Scalar --------------------------------------------------------#

def dVb_dv(p,v):
    return (Omega(v)+sigma(p))*dOmega(v)


##################################################################################################
########################### SATURATION FUNCTIONS #################################################
##################################################################################################

def sigma(x):
    return x/sqrt(1 + x**2.0)

def dsigma(x):
    return 1.0/power(1 + x**2.0, 1.5)

def ddsigma(x):
    return -3.0*x/power(1.0+x**2.0, 2.5)

def sigmaInt(x):
    return sqrt(1.0 + x**2.0) - 1.0;

#--------------------------------------------------------------------------#

def rho(x):
    return sigma(x)

def drho(x):
    return dsigma(x)

def ddrho(x):
    return ddsigma(x)

#--------------------------------------------------------------------------#

def Omega(x):
    if x>2:
        return (3*x**2 - 3*x + 7)/6
    elif x>1:
        return (x**3 - 3*x**2 + 9*x - 1)/6
    else:
        return x

def dOmega(x):
    if x>2:
        return (6*x - 3)/6
    elif x>1:
        return (3*x**2 - 6*x + 9)/6
    else:
        return 1

def ddOmega(x):
    if x>2:
        return 1
    elif x>1:
        return (x - 1)
    else:
        return 0


##################################################################################################
########################### OTHER USED FUNCTIONS #################################################
##################################################################################################

#************************ INPUTS PWM CONVERTION ***************************#


# Comand converter (from 1000 to 2000)
def Cmd_Converter(U,parameters):

    T = U[0]
    w = U[1:4]
    
    # gravity 
    g  = parameters.g

    # mass
    m = parameters.m

    ACRO_RP_P = parameters.ACRO_RP_P;
    
    Max   = ACRO_RP_P*4500/100*pi/180;
    # angular velocity comand between 1000 and 2000 PWM
    U[1] =  1500 + w[0]*500/Max;
    U[2] =  1500 - w[1]*500/Max;
    U[3] =  1500 - w[2]*500/Max;

    # REMARK: the throtle comes between 1000 and 2000 PWM
    # conversion gain
    Throttle_neutral = parameters.Throttle_neutral
    K_THROTLE = m*g/Throttle_neutral;
    U[0] = T/K_THROTLE;

    # rearrage to proper order
    # [roll,pitch,throttle,yaw]
    U_new = zeros(4)
    U_new[0] = U[1]
    U_new[1] = U[2]
    U_new[2] = U[0]
    U_new[3] = U[3]        

    return U_new

#************************ EULERO ANGLES COMPUTATION ***************************#


def GetEulerAngles(R):

    #phi   = atan2(R(3,2),R(3,3));
    #theta = asin(-R(3,1));
    #psi   = atan2(R(2,1),R(1,1));

    EULER = zeros(3)

    sin_theta = -R[2,0]
    sin_theta = bound(sin_theta,1,-1)
    theta     = arcsin(sin_theta)
    EULER[1]  = theta

    sin_phi   = R[2,1]/c(theta)
    sin_phi   = bound(sin_phi,1,-1)
    cos_phi   = R[2,2]/c(theta)
    cos_phi   = bound(cos_phi,1,-1)
    phi       = arctan2(sin_phi,cos_phi)
    EULER[0]  = phi

    sin_psi   = R[1,0]/c(theta)
    sin_psi   = bound(sin_psi,1,-1)
    cos_psi   = R[0,0]/c(theta)
    cos_psi   = bound(cos_psi,1,-1)
    psi       = arctan2(sin_psi,cos_psi)
    EULER[2]  = psi

    # EULER[0] = arctan2(bound(R[2,1],1,-1),bound(R[2,2],1,-1));
    # EULER[1] = arcsin(-bound(R[2,0],1,-1));
    # EULER[2] = arctan2(bound(R[1,0],1,-1),bound(R[0,0],1,-1));    

    return EULER

def bound(x,maxmax,minmin):
    return maximum(minmin,minimum(maxmax,x))


#************************ DERIVATIVE COMPUTATION ***************************#
         

class MatrixDerivative():
    def __init__(self,N,XOld,timeOld):
        self.MatrixFilter = MatrixFilter(N)
        self.XOld = XOld
        self.timeOld = timeOld

    def output(self,timeNew, XNew):
        dt = timeNew - self.timeOld
        dX = XNew - self.XOld
        dXdt =  dX/dt 
        self.XOld = XNew
        self.timeOld = timeNew
        return self.MatrixFilter.output(dXdt)


class MatrixFilter():
    def __init__(self, N):
        self.N = N
        #1st col
        self.Dxx =  MedianFilter(N)
        self.Dyx =  MedianFilter(N)
        self.Dzx =  MedianFilter(N)
        #2nd col
        self.Dxy =  MedianFilter(N)
        self.Dyy =  MedianFilter(N)
        self.Dzy =  MedianFilter(N)
        #3rd col
        self.Dxz =  MedianFilter(N)
        self.Dyz =  MedianFilter(N)
        self.Dzz =  MedianFilter(N)

    def output(self,dataNew):
        #1st col
        DxxNew = self.Dxx.output(dataNew[0,0])
        DyxNew = self.Dyx.output(dataNew[1,0])
        DzxNew = self.Dzx.output(dataNew[2,0])
        #2nd col
        DxyNew = self.Dxy.output(dataNew[0,1])
        DyyNew = self.Dyy.output(dataNew[1,1])
        DzyNew = self.Dzy.output(dataNew[2,1])
        #3rd col
        DxzNew = self.Dxz.output(dataNew[0,2])
        DyzNew = self.Dyz.output(dataNew[1,2])
        DzzNew = self.Dzz.output(dataNew[2,2])

        out = array([DxxNew,DyxNew,DzxNew,DxyNew,DyyNew,DzyNew,DxzNew,DyzNew,DzzNew])

        return reshape(out,(3,3))

class MedianFilter():
    # N is order of median filter
    def __init__(self, N):
        self.N = N
        self.data = numpy.zeros(N)
    
    def update(self,dataNew):
        N = self.N
        self.data[:-1] = self.data[1:]      # data{0:N-2} <- data{1:N-1}      we shift the array to left loosing the oldest element (i=0)
        self.data[-1]  = dataNew           # data{N-1} <- new_data           replace the last element (i=N-1) with the newest
                  
    def output(self,dataNew):
        self.update(dataNew)
        out = numpy.median(self.data)       # middle value of a sorted copy of the vector "self.data" 
        return out                          # (When N is even, it is the average of the two middle values)   

