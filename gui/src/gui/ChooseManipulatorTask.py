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

# import analysis
# import utils
import subprocess



# import services defined in quad_control
# SERVICE BEING USED: Controller_srv
from quad_control.srv import ManipulatorTask
from quad_control.srv import *

# import message of the type controller_state
# because this gui will be able to display the state of the controller
from quad_control.msg import Controller_State


import argparse


class ChooseManipulatorTaskPlugin(Plugin):


    def __init__(self, context,namespace = None):

        # it is either "" or the input given at creation of plugin
        self.namespace = self._parse_args(context.argv())


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

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO SET DESIRED TASK
        self._widget.SetTaskButton.clicked.connect(self.SetTask)

        # BUTTON TO RESET
        self._widget.resetButton.clicked.connect(self.RESET)

        # ------------------------------------------------------------------#
        # ------------------------------------------------------------------#

        # Default values for buttons: 
        self._widget.freq.setValue(1)

       
        # ---------------------------------------------- #
        # ---------------------------------------------- #
        # initial time: this will be used to offset to time to 0 
        # instead of plotting with "real" time
        self.time0 = rospy.get_time()

        # self._widget.GainsOption1.toggled.connect(self.DefaultOptions)
        # self._widget.GainsOption2.toggled.connect(self.DefaultOptions)
        # self._widget.GainsOption3.toggled.connect(self.DefaultOptions)

    # def DefaultOptions(self):

    #     if self._widget.GainsOption1.isChecked():
    #         wn  = 1
    #         xsi = numpy.sqrt(2)/2
    #         ki  = 0.0

    #         wn_z  = 1
    #         xsi_z = numpy.sqrt(2)/2
    #         ki_z  = 0.0

    #         Throttle_neutral = 1400          


    #     kv   = 2.0*xsi*wn
    #     kp   = wn*wn

    #     kv_z   = 2.0*xsi_z*wn_z
    #     kp_z   = wn_z*wn_z


    #     # Default values for buttons: PID Controller
    #     self._widget.PgainXY_PID.setValue(kp)
    #     self._widget.DgainXY_PID.setValue(kv)
    #     self._widget.IgainXY_PID.setValue(ki)
    #     self._widget.PgainZ_PID.setValue(kp_z)
    #     self._widget.DgainZ_PID.setValue(kv_z)
    #     self._widget.IgainZ_PID.setValue(ki_z)
    #     self._widget.ThrottleNeutral_PID.setValue(Throttle_neutral)


    def RESET(self):
        # try: 
        #     # time out of one second for waiting for service
        #     rospy.wait_for_service(self.namespace+'manipulator_task',1.0)
            
        #     try:
        #         ResettingTask = rospy.ServiceProxy(self.namespace+'manipulator_task', ManipulatorTask)

        #         replyR = ResettingTask(0,None)

        #         if replyR.received == True:
        #             self._widget.Success.setChecked(True) 
        #             self._widget.Failure.setChecked(False) 

        #     except rospy.ServiceException:
        #         self._widget.Success.setChecked(False) 
        #         self._widget.Failure.setChecked(True) 
            
        # except:
        #     self._widget.Success.setChecked(False) 
        #     self._widget.Failure.setChecked(True)     
            pass

      

    #@Slot(bool)
    def SetTask(self):

        try: 
            # time out of one second for waiting for service
            rospy.logwarn('\n Waiting for service...')
            rospy.wait_for_service(self.namespace+'manipulator_task',2.0)
            
            try:
                rospy.logwarn('Setting TASK...')
                SettingTask = rospy.ServiceProxy(self.namespace+'manipulator_task', ManipulatorTask)

                if self._widget.TaskSelect.currentIndex() == 1:
                    taskIndex,parameters = self.OscillationParameters()

#                if self._widget.TaskSelect.currentIndex() == 2:
#                    taskIndex,parameters = self.ThrowParameters()

                replyS = SettingTask(taskIndex,parameters)


                if replyS.received == True:
                    self._widget.Success.setChecked(True) 
                    self._widget.Failure.setChecked(False) 
                else:
                    rospy.logwarn('Could not change manipulator task')

            except rospy.ServiceException:
                self._widget.Success.setChecked(False) 
                self._widget.Failure.setChecked(True) 
                rospy.logwarn('Failure: Service not available')
            
        except:
            self._widget.Success.setChecked(False) 
            self._widget.Failure.setChecked(True)    
            rospy.logwarn('Failure: time out')  
            pass     


    def OscillationParameters(self):

        taskIndex = 1
        w = self._widget.freq.value()

        parameters = numpy.array([w])
        
        rospy.logwarn('Manipulator parameters setted '+ w)

        return taskIndex,parameters

    
    # def ReceiveControllerState(self,YesOrNo,parameter):
        

    #     try:

    #         # time out of one second for waiting for service
    #         rospy.wait_for_service(self.namespace+'Controller_State_GUI',1.0)

    #         try:

    #             AskForControllerState = rospy.ServiceProxy(self.namespace+'Controller_State_GUI', Controller_Srv)

    #             reply = AskForControllerState(YesOrNo,parameter)

    #             if reply.received == True:
    #                 # if controller receives message, we know it
    #                 self._widget.Success.setChecked(True) 
    #                 self._widget.Failure.setChecked(False) 
    #             else:
    #                 # if controller does not receive message, we know it
    #                 self._widget.Success.setChecked(False) 
    #                 self._widget.Failure.setChecked(True) 

    #         except rospy.ServiceException:
    #             # print "Service call failed: %s"%e   
    #             self._widget.Success.setChecked(False) 
    #             self._widget.Failure.setChecked(True) 
            
    #     except: 
    #         # print "Service not available ..."        
    #         self._widget.Success.setChecked(False) 
    #         self._widget.Failure.setChecked(True)
    #         pass  


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

    

