#!/usr/bin/env python
# tkinter Stuff
import os
import sys
import copy
if sys.version_info[0] == 3:
    import tkinter as tk
    import tkinter.ttk as ttk
    import tkinter.filedialog as fdialog
else:
    import Tkinter as tk
    import ttk
    import tkFileDialog as fdialog

from ttkthemes import ThemedTk

def _from_rgb(rgb):
    """translates an rgb tuple of int to a tkinter friendly color code
    """
    rgb_int = [0]*3
    for i, color in enumerate(rgb):
        rgb_int[i] = int(color*255)
    return "#%02x%02x%02x" % tuple(rgb_int)



class Spinbox(ttk.Entry):

    def __init__(self, master=None, **kw):

        ttk.Entry.__init__(self, master, "ttk::spinbox", **kw)
    def set(self, value):
        self.tk.call(self._w, "set", value)


class OptionSwitcher(ttk.OptionMenu):
    def __init__(self, container, variable, default=None, *values, **kwargs):
        fill_option = "< Choose >"
        values_cp = copy.deepcopy(*values)
        values_cp.insert(0, fill_option)
        if default is None:
            default = values_cp[0]

        ttk.OptionMenu.__init__(self, container,variable, default, *values_cp, **kwargs)