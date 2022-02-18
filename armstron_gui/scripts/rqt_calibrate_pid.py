#!/usr/bin/env python

import sys

from armstron_gui.calibrate_pid import MyPlugin
from rqt_gui.main import Main

plugin = 'armstron_gui.calibrate_pid'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))