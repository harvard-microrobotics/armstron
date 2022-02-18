#!/usr/bin/env python

import sys

from armstron_gui.set_pressures_adv import MyPlugin
from rqt_gui.main import Main

plugin = 'armstron_gui.set_pressures_adv'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))