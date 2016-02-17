#!/usr/bin/env python

import rospy
import numpy
from numpy import *
from numpy.linalg import *

from SomeFunctions import OP,skew,GetRotFromEulerAnglesDeg
from SomeFunctions import GetRotFromEulerAnglesDeg,Velocity_Filter

from quad_control.msg import quad_state_and_cmd
from quad_control.msg import load_state

from quad_control.srv import TrajDes_Srv
from quad_control.srv import TaskStart
from quad_control.srv import TaskStop
from quad_control.srv import Manipulator
from quad_control.srv import LoadSrv
from quad_control.srv import *

# when Mocap is used this is necessary
import mocap_source

from std_srvs.srv import Empty

CLOSE = True
OPEN = False
VERTICAL = 88

BODY_JOINT = 0.07   # 10cm
JOINT_CM = 0.05     # 5cm
CM_EEF = 0.12       # 12cm

offsetHovering = 0.30   #40cm
#zOffestFly = 1.2

massManip = 0.300
massLoad = 0
m = 1.442



class ManipulationTaskPlanner:

    def __init__(self):


        self.taskON = False
        self.target = ones(3)
        self.goalXY = ones(3)
        self.errorGrasping = 0.03
        self.errorHovering = 0.10

        self.namespace = ''

        self.flyingAltitude = 0.8   # meters
        self.target_A = ones(3)
        self.target_B = ones(3)
        self.massLoad = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.posQuad = zeros(3)
        self.posCM = zeros(3)
        self.posEEF = zeros(3)
        self.R = eye(3)
        self.jointAngle = 0

        self.ctXY = 0
        self.ctZ = 0
        self.goDownFlag = False

    def handle_start(self, req):

        self.taskON = True      # flag: task is running
        self.ctXY = 0

        self.namespace = '/'+req.namespace

        self.target = self.getPosLoad()    #req.targetA    # EEF target 

        self.massLoad = req.massLoad
        self.errorGrasping = req.tolleranceGripper
        self.errorHovering = 0.10   # 15cm
        self.flyingAltitude = req.flyingAltitude

        self.subscriberState = rospy.Subscriber(self.namespace+'quad_state_and_cmd', quad_state_and_cmd, self.callback_getQuadState)

        self.runTask()

        return []

    def handle_stop(self, req=None):
        self.taskON =False
        return []

    def runTask(self):

        #self.takeOff()  
        # wait ...
        self.setPose(OPEN,VERTICAL)
        target = self.target
        targetXY = target + array([0,0, self.flyingAltitude]) # array([target[0], target[1], self.z])
        self.goalXY = self.EEF2CM(targetXY) 
        self.setNextPoint(self.goalXY)     # coordinates of CM


    def takeOff(self):
        self.SaveHome()
        self.GoUp()

    def GoUp(self):
        x  = self.x
        y  = self.y
        z  = self.z + self.flyingAltitude
        self.setNextPoint(array([x,y,z]))   # coordinates of CM

    def SaveHome(self):
        self.xHome  = self.x
        self.yHome  = self.y
        self.zHome  = self.z

    def GoHome(self):
        x  = self.xHome
        y  = self.yHome
        z  = self.z
        self.setNextPoint(array([x,y,z]))    # coordinates of CM

    # send the desired point where to move:
    # we use the service "TrajDes_GUI" to set the desired point 
    # REMEMBER: when is attached the manipulator, 
    #           the reference position is for the global CM 
    #           and not only for the quad CM
    def setNextPoint(self, coordinatesCM):
        traj = 0  # select "traj_des_still" 
        offset = coordinatesCM               
        rotation = zeros(3)
        parameters = None

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'TrajDes_GUI', 1.0)
            serviceTrajDes = rospy.ServiceProxy(self.namespace+'TrajDes_GUI', TrajDes_Srv)

            reply = serviceTrajDes(traj,offset,rotation,parameters)

            if reply.received == True:
                rospy.logwarn('New Point Setted')

        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('Failure: Service not available')


    def goDown(self, coordinatesCM):
        traj = 2  # select "traj_des_trapez_vertical" 

        xCM = coordinatesCM[0]
        yCM = coordinatesCM[1]
        zCM = coordinatesCM[2]

        tc = 3
        tf = 10
        ad = 0.01
        zi = self.z
        zf = zCM

        offset = array([xCM, yCM,0])               
        rotation = zeros(3)
        parameters = array([tc,tf,ad,zi,zf])

        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'TrajDes_GUI', 1.0)
            serviceTrajDes = rospy.ServiceProxy(self.namespace+'TrajDes_GUI', TrajDes_Srv)

            reply = serviceTrajDes(traj,offset,rotation,parameters)

            if reply.received == True:
                rospy.logwarn('New Point Setted')

        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('Failure: Service not available')


    # set the manipulator pose:
    # we use the service "manipualtor_command" to set the open/close the gripper and move the joint
    def setPose(self, closeGripper, jointAngle):

        self.jointAngle = jointAngle

        try:
            # it waits for service for 2 seconds
            #rospy.logwarn('Waiting for service...')
            rospy.wait_for_service(self.namespace+'manipulator_commands', 2.0) 
            #rospy.logwarn('Setting MANIPULATOR...')
            change_state = rospy.ServiceProxy(self.namespace+'manipulator_commands', Manipulator)

            manipulator_resp = change_state(closeGripper, jointAngle)

        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('manipulator_commands Failure: Service not available')

    # set the mass load:
    # we use the service "set_load" to set the mass of the load
    def setLoad(self,massLoad):
        try:
            # it waits for service for 2 seconds
            rospy.wait_for_service(self.namespace+'change_load', 1.0) 
            change_load = rospy.ServiceProxy(self.namespace+'change_load', LoadSrv)
            loadSrvResponce = change_load(massLoad)

            if loadSrvResponce.success:
                rospy.logwarn('Load mass = '+ str(massLoad))
            else:
                rospy.logwarn('Could not change load parameters')

        except rospy.ROSException, rospy.ServiceException:
                rospy.logwarn('change_load Failure: Service not available')


    # CALLBACK: update the state of the quad.
    # we subscribe to the topic "quad_state_and_cmd" to get 'state' and 'state_d'
    def callback_getQuadState(self,data):

        self.posQuad = array([data.x, data.y, data.z])
        self.posCM = self.QUAD2CM(self.posQuad)
        self.posEEF = self.QUAD2EEF(self.posQuad)

        self.x = self.posCM[0]
        self.y = self.posCM[1]
        self.z = self.posCM[2]

        ee = array([data.roll, data.pitch, data.yaw])
        RR  = GetRotFromEulerAnglesDeg(ee)
        self.R = reshape(RR,(3,3))

    #################### Coordinates Transformation Functions ###################

    def EEF2CM(self, posEEF):
        posQuad = self.EEF2QUAD(posEEF)
        posCM = self.QUAD2CM(posQuad)
        return posCM

    def EEF2QUAD(self, posEEF):
        R = self.R
        posEEFwrtBody = self.EEFwrtBody()
        posQuad = posEEF + R.dot(posEEFwrtBody)
        return posQuad

    def QUAD2EEF(self, posQuad):
        R = self.R
        posEEFwrtBody = self.EEFwrtBody()
        posEEF = posQuad - R.dot(posEEFwrtBody)
        return posEEF

    def QUAD2CM(self, posQuad):
        R = self.R
        ManipCMwrtBody = self.ManipCMwrtBody()
        EEFwrtBody = self.EEFwrtBody()
        posCM = posQuad - (massManip/m)*R.dot(ManipCMwrtBody) - (massLoad/m)*R.dot(EEFwrtBody)
        return posCM

    def EEFwrtBody(self):
        gamma = self.jointAngle
        out = (BODY_JOINT + JOINT_CM + CM_EEF)*array([0, 0, 1])
        #out = BODY_JOINT*array([0, 0, 1]) + (JOINT_CM + CM_EEF)*array([cos(gamma), 0, sin(gamma)])
        return out

    def ManipCMwrtBody(self):
        gamma = self.jointAngle
        out = (BODY_JOINT + JOINT_CM)*array([0, 0, 1])
        #out = BODY_JOINT*array([0, 0, 1]) + (JOINT_CM)*array([cos(gamma), 0, sin(gamma)])
        return out

    # HANDLE: update the new desired targets 'A' and 'B'
    #def handle_DesiredTargets():


    ########################## Qualisys #######################################

    def getPosLoad(self):
        if self.flagMOCAP == True:
            bodies = self.Qs.get_body(self.body_id)
            if bodies != 'off':  
                x=bodies["x"] 
                y=bodies["y"] - 0.03
                z=bodies["z"]   
                # position
                posLoad = array([x,y,z])
                return posLoad 
            else:
                pass
        else:
            pass


    #callback for turning ON/OFF Mocap and turning OFF/ON the subscription to the simulator
    def handle_Mocap(self,req):
        # if mocap is turned on
        if self.flagMOCAP_On == True:
            # request to turn OFF Mocap, and turn on subscription to Simulator messages
            if req.On == False:
                # in case Qs is not defined yet
                try: 
                    # close mocap connection
                    # self.Qs._stop_measurement()
                    del self.Qs
                    self.flagMOCAP_On = False
                    # set flag to OFF
                    self.flagMOCAP = False
                    # service has been provided
                    return Mocap_IdResponse(True,True)
                except:
                    # service was NOT provided
                    return Mocap_IdResponse(True,False) 

            # if we are requested to change the body Id
            if req.change_id == True:
                # see list of available bodies
                bodies = self.Qs.get_updated_bodies()
                # check if body_id available
                body_indice = -1
                # Get the corresponding id of the body
                if isinstance(bodies,list):
                    for i in range(0,len(bodies)):
                        rospy.logwarn(bodies[i]['id'])
                        if(bodies[i]['id']==req.id):
                            body_indice=i
                # save body id
                self.body_id = req.id                        
                if body_indice == -1:
                    # body does not exist
                    self.flagMOCAP = False
                    # body does not exist, but service was provided
                    return Mocap_IdResponse(False,True)
                else:
                    # body exists
                    # set flag to on
                    self.flagMOCAP = True
                    # body EXISTS, and service was provided
                    return Mocap_IdResponse(True,True)
        else:
            # if Mocap is turned off, and we are requested to turn it on
            if req.On == True:
                # establish connection to qualisys
                self.Qs = mocap_source.Mocap(info=0)
                self.flagMOCAP_On = True
                # service was provided
                return Mocap_IdResponse(False,True)


    # return list of bodies detected by mocap or numbers 1 to 99 if not available
    def handle_available_bodies(self, dummy):
        if self.flagMOCAP_On:
            try: # sometimes mocap causes unpredictable errors
                bodies = self.Qs.find_available_bodies(False)
                if len(bodies) > 0:
                    return {"bodies": bodies[0]}
            except:
                pass
        return {"bodies": range(0,100)}


    ###################################################
    #################   MAIN LOOP   ###################
    ###################################################

    def wait_task(self):

        rospy.init_node('manipulation_task_planner')

        rospy.Service('task_start', TaskStart, self.handle_start)
        rospy.Service('task_stop', TaskStop, self.handle_stop)

        # flag for MOCAP is initialized as FALSE
        self.flagMOCAP    = False
        self.flagMOCAP_On = False
        Save_MOCAP_service = rospy.Service('Mocap_Set_Id_Load', Mocap_Id, self.handle_Mocap)
        mocap_available_bodies = rospy.Service('MocapBodies_Load', MocapBodies, self.handle_available_bodies)


        self.pubPosLoad = rospy.Publisher('load_state', load_state, queue_size=10)

        self.pubPosLoad.publish(self.getPosLoad())
        rospy.logwarn(str(self.getPosLoad()))

        rate = rospy.Rate(90)

        while not rospy.is_shutdown():


            if self.taskON:   

                self.pubPosLoad.publish(self.getPosLoad())
                rospy.logwarn(str(self.getPosLoad()))

                # if self.errorGrasping < 0:
                #     self.setPose(CLOSE,0)
                #     rospy.sleep(0.8)            
                #     self.setPose(CLOSE,180)
                #     rospy.sleep(0.8)

                # -------------- Move to the XY position of the target --------------
                errorCM = norm(self.posCM - self.goalXY)
                # rospy.logwarn(str(errorCM))

                # if errorCM < self.errorHovering:
                #     self.ctXY = self.ctXY +1
                #     if  self.ctXY == 5
                #         target = self.target
                #         goalZ = self.EEF2CM(target)
                #         self.goDown(goalZ)
                # else 
                #     self.ctXY = 0

                if errorCM < self.errorHovering:
                    self.ctXY = self.ctXY +1
                    if  self.ctXY == 10:
                        self.goDownFlag = True
                else:
                    self.ctXY = 0

                # errorCM = norm(self.posCM - self.goalZ)

                if self.goDownFlag and 0.1*self.ctZ<=self.flyingAltitude:
                    target = self.target + array([0,0,self.flyingAltitude-0.1*self.ctZ])
                    self.goalZ = self.EEF2CM(target)
                    self.setNextPoint(self.goalZ)

                    self.ctZ = self.ctZ + 1
                    self.goalXY = self.goalZ
                    self.goDownFlag = False

                # -------------- Move to the Z position of the target --------------
                errorEEF = norm(self.posEEF - self.target)
                # rospy.logwarn(str(errorEEF))

                if errorEEF < self.errorGrasping:
                    self.setPose(CLOSE,VERTICAL)
                    self.setLoad(self.massLoad)
                    rospy.sleep(0.5)    # wait 0.5s for close the gripper
                    self.GoUp()
                    self.taskON = False

            rate.sleep() 


if __name__ == '__main__':
    
    task = ManipulationTaskPlanner()
    
    try:
        task.wait_task()

    except rospy.ROSInterruptException:
        pass