#!/usr/bin/env python

import sys

from armstron_gui.set_pressures_basic import MyPlugin
from rqt_gui.main import Main

plugin = 'armstron_gui.set_pressures_basic'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))