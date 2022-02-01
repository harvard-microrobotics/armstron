#!/usr/bin/env python

import sys

from virtual_instron_gui.calibrate_valves import MyPlugin
from rqt_gui.main import Main

plugin = 'virtual_instron_gui.calibrate_valves'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))