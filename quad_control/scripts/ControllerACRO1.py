#!/usr/bin/env python
# this line is just used to define the type of document

import rospy

import numpy
from numpy import *
from numpy.linalg import *
from numpy import cos as c
from numpy import sin as s

from SomeFunctions import OP,skew,GetRotFromEulerAnglesDeg

from Controllers_Parameters import parameters_sys


class ControllerThrustOmega():
    
    # DEFAULT PARAMETERS
    parameters = parameters_sys()

    # this is a static controller, but I include this here for convenience
    # so that all controllers have an estimated disturbance
    d_est      = zeros(3)



    def __init__(self,parameters = None):
        if parameters != None:
            self.parameters.ks   = parameters[0]
            self.parameters.katt  = parameters[1]
            self.parameters.kb  = parameters[1]



    def update_parameters(self,parameters):
        self.parameters = parameters



    def output(self,t,states,states_d):

        # convert euler angles from deg to rotation matrix
        ee = states[6:9]
        R  = GetRotFromEulerAnglesDeg(ee)
        R  = reshape(R,9)

        # collecting states
        states  = concatenate([states[0:6],R])
        return controller(states,states_d,self.parameters)



##################################################################################################
########################### CONTROLLER ###########################################################
##################################################################################################

def controller(states,states_d,parameters):
    
    # ****************** PARAMETERS ************************#

    # mass of vehicles (kg)
    m = parameters.m
    
    # acceleration due to gravity (m/s^2)
    g  = parameters.g
    
    # third canonical basis vector
    e3 = array([0.0, 0.0, 1.0])
    
    # position and velocity
    x  = states[0:3];
    v  = states[3:6];

    # thrust unit vector and its angular velocity
    R  = states[6:15];
    R  = reshape(R,(3,3))
    n  = R.dot(e3)

    # desired quad trajectory
    xd = states_d[0:3];
    vd = states_d[3:6];
    ad = states_d[6:9];
    jd = states_d[9:12];
    sd = states_d[12:15];
    
    # position error and velocity error
    ep = x - xd
    ev = v - vd


    # ****************** CONTROL INPUTS ************************#
    
    # -----------------------Thrust --------------------------#
    u = ad + g*e3 + uBounded(ep, ev, parameters)

    #throttle
    T = dot(u,n)    # T = u'*n     

    #thrust
    Thrust = m*T


    # ------------------Angular Velocity ---------------------#
    
    # acceleration error
    ea = T*n - g*e3 - ad

    # derivative of "u"
    uDot = jd + uBoundedDot(ep, ev, ea, parameters)

    # desired attitude
    nd = u/norm(u)                                       

    # desired angular velocity 
    RtrSnd = dot(transpose(R),skew(nd))                         # R'*skew(nd)                      
    wd = dot(RtrSnd,uDot)/norm(u)                               # wd = (R'*skew(nd)*uDot)/norm(u)

    # partial derivative lyapunov function 
    dV2 = dV2Backstepping(ep, ev)

    # angular velocity    
    ks = parameters.ks; 
    katt = parameters.katt;
    RtrSn = dot(transpose(R),skew(n)) 
    w = wd - ks*norm(u)*dot(RtrSn,dV2) - katt*dot(RtrSnd,n)  # w = wd-ks*norm(u)*(R'*skew(n)*dV2)-katt*(R'*skew(nd)*n)


    # ------------------ Control Inputs Vector ------------------#

    U = zeros(4)
    
    U[0] = Thrust
    U[1:4] = w


    #??????????????????????????????????????????????????????????????????????????
    #??????????????????????????????????????????????????????????????????????????
    # # yaw control: gain
    # k_yaw    = parameters.k_yaw; 
    # # desired yaw: psi_star
    # psi_star = parameters.psi_star; 
    
    # #current euler angles
    # euler = GetEulerAngles(R);
    # phi   = euler[0];
    # theta = euler[1];
    # psi   = euler[2];

    # psi_star_dot = 0;
    # psi_dot  = psi_star_dot - k_yaw*s(psi - psi_star);
    # U[3]    = 1/c(phi)*(c(theta)*psi_dot - s(phi)*U[2]);
    #??????????????????????????????????????????????????????????????????????????
    #??????????????????????????????????????????????????????????????????????????

    U = Cmd_Converter(U, parameters)

    return U


################################# BOUNDED INPUT ####################################################


# ------------------------------ Vectorial --------------------------------------------------------#

def uBounded(ep, ev, parameters):
    out   = zeros(3)
    out[0] = uBounded_Scalar(ep[0], ev[0], parameters)
    out[1] = uBounded_Scalar(ep[1], ev[1], parameters)
    out[2] = uBounded_Scalar(ep[2], ev[2], parameters)
    return out

def uBoundedDot(ep, ev, ea, parameters):
    out  = zeros(3)
    out[0] = uBoundedDot_Scalar(ep[0], ev[0], ev[0], ea[0], parameters)
    out[1] = uBoundedDot_Scalar(ep[1], ev[1], ev[1], ea[1], parameters)
    out[2] = uBoundedDot_Scalar(ep[2], ev[2], ev[2], ea[2], parameters)
    return out


# ------------------------------ Scalar --------------------------------------------------------#

def uBounded_Scalar(p, v, parameters):

    kb = parameters.kb; 
    rospy.loginfo("dOmega=%s, Omega+sigma=%s" % (dOmega(v), Omega(v)+sigma(p)))

    return - kb*(sigma(p)/dOmega(v))*(v+sigma(p))/(Omega(v)+sigma(p)) - v*dsigma(p)/dOmega(v) - rho(Omega(v)+sigma(p));


def uBoundedDot_Scalar(p, v, pDot, vDot, parameters):

    kb = parameters.kb; 

    dr = drho(Omega(v)+sigma(p))

    Om = Omega(v)
    dOm = dOmega(v)
    ddOm = ddOmega(v)

    s = sigma(p)
    ds = dsigma(p)
    dds = ddsigma(p)

    sOm = s + Om
    sv = s + v
    s2v = 2*s + v

    num1 = dOm*(-kb*(s**2+s2v*Om)-sOm**2*dr*dOm)
    num2 = -v*dds*sOm**2
    den = dOm*sOm**2
    DubDp =  (num1 + num2)/den

    a1 = ds/(dOm**2)
    b1 = v*ddOm-dOm
    term1 = a1*b1
    a2 = kb*s/(sOm*dOm)**2
    b2 = sv*(sOm*ddOm + dOm**2) - sOm*dOm 
    term2 = a2*b2
    term3 = -dr*dOm
    DubDv =  term1 + term2 + term3

    return DubDp*pDot + DubDv*vDot


##################################################################################################


def GetEulerAngles(R):

    #phi   = atan2(R(3,2),R(3,3));
    #theta = asin(-R(3,1));
    #psi   = atan2(R(2,1),R(1,1));

    EULER = zeros(3)

    sin_theta = -R[2,0]
    sin_theta = bound(sin_theta,1,-1)
    theta     = arcsin(sin_theta)
    EULER[1]      = theta

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


##################################### PWM INPUTS #######################################################


# Comand converter (from 1000 to 2000)
def Cmd_Converter(U,parameters):

    T = U[0]
    w = U[1:4]

    # mass of vehicles (kg)
    m = parameters.m
    
    # acceleration due to gravity (m/s^2)
    g  = parameters.g

    ACRO_RP_P = parameters.ACRO_RP_P;
    
    Max   = ACRO_RP_P*4500/100*pi/180;
    # angular velocity comand between 1000 and 2000 PWM
    U[1] =  1500 + w[0]*500/Max;
    U[2] =  1500 - w[1]*500/Max;
    U[3] =  1500 - w[2]*500/Max;
    

    # REMARK: the throtle comes between 1000 and 2000 PWM
    # conversion gain
    Throttle_neutral = parameters.Throttle_neutral;
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


##################################################################################################
########################### LYAPUNOV DERIVATIVES #################################################
##################################################################################################

# ------------------------------ Vectorial --------------------------------------------------------#

def dV2Backstepping(ep, ev):
    out = zeros(3)
    out[0] = dVbBackstepping(ep[0], ev[0])
    out[1] = dVbBackstepping(ep[1], ev[1])
    out[2] = dVbBackstepping(ep[2], ev[2])
    return out

# ------------------------------ Scalar --------------------------------------------------------#

def dVbBackstepping(p,v):
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
        return sign(x)*(3*x**2 - 3*x + 7)/6
    elif x>1:
        return sign(x)*(x**3 - 3*x**2 + 9*x - 1)/6
    else:
        return x

def dOmega(x):
    if x>2:
        return sign(x)*(6*x - 3)/6
    elif x>1:
        return sign(x)*(3*x**2 - 6*x + 9)/6
    else:
        return 1

def ddOmega(x):
    if x>2:
        return sign(x)
    elif x>1:
        return sign(x)*(x - 1)
    else:
        return 0

