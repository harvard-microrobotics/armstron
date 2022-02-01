#!/usr/bin/env python

import sys

from virtual_instron_gui.set_pressures_basic import MyPlugin
from rqt_gui.main import Main

plugin = 'virtual_instron_gui.set_pressures_basic'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))