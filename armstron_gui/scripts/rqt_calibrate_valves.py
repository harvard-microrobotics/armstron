#!/usr/bin/env python

import sys

from armstron_gui.calibrate_valves import MyPlugin
from rqt_gui.main import Main

plugin = 'armstron_gui.calibrate_valves'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))