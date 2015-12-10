#!/usr/bin/env python

import sys

from gui.ChooseManipulatorTask import ChooseManipulatorTask
from rqt_gui.main import Main

plugin = 'ChooseManipulatorTask'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
