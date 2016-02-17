import os
import rospy
import QtGui
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4.QtCore import QObject, pyqtSignal
import PyQt4.QtCore
import pyqtgraph as pg

# necessary to have gui as a client, asking controller to save data
# from python_qt_binding.QtCore import QTimer, Slot
# from python_qt_binding.QtCore import pyqtSlot



import numpy
from numpy import *

# import analysis
# import utils
import subprocess



# import services defined in quad_control
# SERVICE BEING USED: Controller_srv
from quad_control.srv import TaskStart
from quad_control.srv import TaskStop
from quad_control.srv import Manipulator
from quad_control.srv import *

# import message of the type controller_state
# because this gui will be able to display the state of the controller
from quad_control.msg import Controller_State


import argparse


class ChooseManipulatorTaskPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())               # 'Iris1/' different to '/Iris1/'
        #rospy.logwarn('namespace[ChooseManipulatorTask] = '+self.namespace)

        super(ChooseManipulatorTaskPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChooseManipulatorTaskPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ChooseManipulatorTask.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ChooseManipulatorTaskUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #################################  BUTTONs  ##############################################

        # BUTTON TO SET DESIRED TASK
        self._widget.StartTaskButton.clicked.connect(self.start)

        # BUTTON TO RESET
        self._widget.ResetButton.clicked.connect(self.reset)

        # initial time: this will be used to offset to time to 0 
        # instead of plotting with "real" time
        self.time0 = rospy.get_time()

        # self._widget.GainsOption1.toggled.connect(self.DefaultOptions)
        # self._widget.GainsOption2.toggled.connect(self.DefaultOptions)
        # self._widget.GainsOption3.toggled.connect(self.DefaultOptions)

        # ************************* QUALISYS *****************************

        # BUTTON TO CONNECT TO QUALISYS
        self._widget.Qualisys.stateChanged.connect(self.QualisysConnect)
        self._widget.QualisysTarget.stateChanged.connect(self.QualisysConnectTarget)

        # BUTTON TO CHANGE ID
        self._widget.changeID.clicked.connect(self.changeID)
        self._widget.changeIDTarget.clicked.connect(self.changeIDTarget)

        # Default Value for Id of Quad (as in QUALISYS computer)
        self.set_up_body_list()
        self.set_up_body_list_target()


    def reset(self): 
        try: 
            rospy.wait_for_service(self.namespace+'task_stop',2.0)
            serviceStop = rospy.ServiceProxy(self.namespace+'task_stop', TaskStop)

            serviceStop()
            
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('task_stop Failure') 
      

    #@Slot(bool)
    def start(self):

        xA = self._widget.xA.value()
        yA = self._widget.yA.value()
        zA = self._widget.zA.value()
        target_A = array([xA,yA,zA])

        xB = self._widget.xB.value()
        yB = self._widget.yB.value()
        zB = self._widget.zB.value()
        target_B = array([xB,yB,zB])

        massLoad = self._widget.massLoad.value()
        tollerance = self._widget.tollerance.value()
        flyingAltitude = self._widget.flyingAltitude.value()


        try: 
            rospy.wait_for_service(self.namespace+'task_start',2.0)
            serviceTask = rospy.ServiceProxy(self.namespace+'task_start', TaskStart)

            serviceTask(self.namespace,target_A,target_B,massLoad,tollerance,flyingAltitude)
            
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn('task_start Failure')  



    def _parse_args(self, argv):

        parser = argparse.ArgumentParser(prog='saver', add_help=False)

        # args = parser.parse_args(argv)

        if argv:
            namespace = argv[0]
            return namespace            
        else:
            # is argv is empty return empty string
            return ""
    
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog



##################################################################################################
####################################### QUALISYS #################################################
##################################################################################################



 # ***************************************** load *****************************************************

    def set_up_body_list(self):
        # try to obtain list of available mocap bodies
        try:
            rospy.wait_for_service(self.namespace+'MocapBodies', 1.)
            req_bodies = rospy.ServiceProxy(self.namespace+'MocapBodies', MocapBodies)
            bodies = req_bodies().bodies
        except:
            bodies = range(0,100)

        preferred_index = self._widget.MocapID.currentText()

        # clear existing entries
        # this may trigger the following bug in QT:
        # https://bugreports.qt.io/browse/QTBUG-13925
        self._widget.MocapID.clear()
        # convert bodies to string list and add as list to entries
        self._widget.MocapID.insertItems(0, map(str, bodies))

        # try to set new index so the value does not change
        try:
            new_index = bodies.index(int(preferred_index))
            self._widget.MocapID.setCurrentIndex(new_index)
        except:
            pass
    
    #@Slot(bool)
    def QualisysConnect(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id_Load',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id_Load', Mocap_Id)

                if self._widget.Qualisys.isChecked() == True:
                    reply = AskMocap(True,int(self._widget.MocapID.currentText()),True)

                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False) 
                    if reply.exists == True:
                        self._widget.Exists.setChecked(True)
                        self._widget.ExistsNot.setChecked(False)
                    else:
                        self._widget.Exists.setChecked(False)
                        self._widget.ExistsNot.setChecked(True)
                else:
                    reply = AskMocap(False,int(self._widget.MocapID.currentText()),True)
                    
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False) 

                    self._widget.Exists.setChecked(False)
                    self._widget.ExistsNot.setChecked(False)


            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccess.setChecked(False) 
                self._widget.QualisysFailure.setChecked(True) 
                self._widget.Exists.setChecked(False)
                self._widget.ExistsNot.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccess.setChecked(False) 
            self._widget.QualisysFailure.setChecked(True)
            self._widget.Exists.setChecked(False)
            self._widget.ExistsNot.setChecked(False)            
            pass

        # initialize list of available bodies
        self.set_up_body_list()


    #@Slot(bool)
    def changeID(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id_Load',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id_Load', Mocap_Id)

                if self._widget.Qualisys.isChecked() == True:
                    reply = AskMocap(True,int(self._widget.MocapID.currentText()),True)
                    print(reply)
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccess.setChecked(True) 
                        self._widget.QualisysFailure.setChecked(False)

                    if reply.exists == True:
                        self._widget.Exists.setChecked(True)
                        self._widget.ExistsNot.setChecked(False)
                    else:
                        self._widget.Exists.setChecked(False)
                        self._widget.ExistsNot.setChecked(True)

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccess.setChecked(False) 
                self._widget.QualisysFailure.setChecked(True) 
                self._widget.Exists.setChecked(False)
                self._widget.ExistsNot.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccess.setChecked(False) 
            self._widget.QualisysFailure.setChecked(True)
            self._widget.Exists.setChecked(False)
            self._widget.ExistsNot.setChecked(False)            
            pass


 # ***************************************** target *****************************************************


    def set_up_body_list_target(self):
        # try to obtain list of available mocap bodies
        try:
            rospy.wait_for_service(self.namespace+'MocapBodies', 1.)
            req_bodies = rospy.ServiceProxy(self.namespace+'MocapBodies', MocapBodies)
            bodies = req_bodies().bodies
        except:
            bodies = range(0,100)

        preferred_index = self._widget.MocapIDTarget.currentText()

        # clear existing entries
        # this may trigger the following bug in QT:
        # https://bugreports.qt.io/browse/QTBUG-13925
        self._widget.MocapIDTarget.clear()
        # convert bodies to string list and add as list to entries
        self._widget.MocapIDTarget.insertItems(0, map(str, bodies))

        # try to set new index so the value does not change
        try:
            new_index = bodies.index(int(preferred_index))
            self._widget.MocapIDTarget.setCurrentIndex(new_index)
        except:
            pass

  
    #@Slot(bool)
    def QualisysConnectTarget(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id_Target',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id_Target', Mocap_Id)

                if self._widget.QualisysTarget.isChecked() == True:
                    reply = AskMocap(True,int(self._widget.MocapIDTarget.currentText()),True)

                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccessTarget.setChecked(True) 
                        self._widget.QualisysFailureTarget.setChecked(False) 
                    if reply.exists == True:
                        self._widget.ExistsTarget.setChecked(True)
                        self._widget.ExistsNotTarget.setChecked(False)
                    else:
                        self._widget.ExistsTarget.setChecked(False)
                        self._widget.ExistsNotTarget.setChecked(True)
                else:
                    reply = AskMocap(False,int(self._widget.MocapIDTarget.currentText()),True)
                    
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccessTarget.setChecked(True) 
                        self._widget.QualisysFailureTarget.setChecked(False) 

                    self._widget.ExistsTarget.setChecked(False)
                    self._widget.ExistsNotTarget.setChecked(False)


            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccessTarget.setChecked(False) 
                self._widget.QualisysFailureTarget.setChecked(True) 
                self._widget.ExistsTarget.setChecked(False)
                self._widget.ExistsNotTarget.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccessTarget.setChecked(False) 
            self._widget.QualisysFailureTarget.setChecked(True)
            self._widget.ExistsTarget.setChecked(False)
            self._widget.ExistsNotTarget.setChecked(False)            
            pass

        # initialize list of available bodies
        self.set_up_body_list()


    #@Slot(bool)
    def changeIDTarget(self):
        
        try: 
            # time out of one second for waiting for service
            rospy.wait_for_service(self.namespace+'Mocap_Set_Id_Target',1.0)
            
            try:
                AskMocap = rospy.ServiceProxy(self.namespace+'Mocap_Set_Id_Target', Mocap_Id)

                if self._widget.QualisysTarget.isChecked() == True:
                    reply = AskMocap(True,int(self._widget.MocapIDTarget.currentText()),True)
                    print(reply)
                    if reply.success == True:
                        # if controller receives message, we know it
                        self._widget.QualisysSuccessTarget.setChecked(True) 
                        self._widget.QualisysFailureTarget.setChecked(False)

                    if reply.exists == True:
                        self._widget.ExistsTarget.setChecked(True)
                        self._widget.ExistsNotTarget.setChecked(False)
                    else:
                        self._widget.ExistsTarget.setChecked(False)
                        self._widget.ExistsNotTarget.setChecked(True)

            except rospy.ServiceException:
                # print "Service call failed: %s"%e   
                self._widget.QualisysSuccessTarget.setChecked(False) 
                self._widget.QualisysFailureTarget.setChecked(True) 
                self._widget.ExistsTarget.setChecked(False)
                self._widget.ExistsNotTarget.setChecked(False)
            
        except: 
            # print "Service not available ..."        
            self._widget.QualisysSuccessTarget.setChecked(False) 
            self._widget.QualisysFailureTarget.setChecked(True)
            self._widget.ExistsTarget.setChecked(False)
            self._widget.ExistsNotTarget.setChecked(False)            
            pass
