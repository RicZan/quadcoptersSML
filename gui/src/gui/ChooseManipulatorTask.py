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

        # ---------------------------------------------- #
        # ---------------------------------------------- #

        # BUTTON TO SET DESIRED TASK
        self._widget.StartTaskButton.clicked.connect(self.start)

        # BUTTON TO RESET
        self._widget.ResetButton.clicked.connect(self.reset)

        # ------------------------------------------------------------------#
        # ------------------------------------------------------------------#
       
        # ---------------------------------------------- #
        # ---------------------------------------------- #
        # initial time: this will be used to offset to time to 0 
        # instead of plotting with "real" time
        self.time0 = rospy.get_time()

        # self._widget.GainsOption1.toggled.connect(self.DefaultOptions)
        # self._widget.GainsOption2.toggled.connect(self.DefaultOptions)
        # self._widget.GainsOption3.toggled.connect(self.DefaultOptions)


    def reset(self):  
        pass
      

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

            serviceTask(self.namespace,target_A,massLoad,tollerance,flyingAltitude)
            
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

    

