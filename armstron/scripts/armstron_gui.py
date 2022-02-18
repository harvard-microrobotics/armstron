#!/usr/bin/env python
# tkinter Stuff
import tkinter as tk
import tkinter.ttk as ttk
from ttkthemes import ThemedTk
import tkinter.filedialog as fdialog
import os
import sys
import copy
import time
import threading

# ROS Stuff
#import armstron.utils as utils
#import rospy
#import rospkg
#import actionlib
#import json
#import armstron.msg as msg
#from armstron.srv import Balance, Estop
#filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')
filepath_config = '../config'

sys.path.append("../src/armstron")
import utils 



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
        print(values_cp)
        values_cp.insert(0, fill_option)
        if default is None:
            default = values_cp[0]

        ttk.OptionMenu.__init__(self, container,variable, default, *values_cp, **kwargs)



class ArmstronControlGui:
    def __init__(self):
        self.config_folder=filepath_config
        self.load_settings()
        self.default_option = "< Choose >"


    def load_settings(self, filename="default.yaml"):
        self.settings = utils.load_yaml(os.path.join(self.config_folder,'gui',filename))

        self.test_profile_path = os.path.join(self.config_folder,'test_profiles')
        self.test_profile_options = [f for f in os.listdir(self.test_profile_path) if os.path.isfile(os.path.join(self.test_profile_path, f))]

        self.file_types = self.settings['file_types']
        self.color_scheme = self.settings['color_scheme']

    def init_gui(self):
        # Make a new window
        self.root = ThemedTk(theme="breeze")#tk.Tk()
        self.root.title("Pressure Control Interface")

        # Add the statusbar
        self.status_bar = tk.Label(self.root, text="Hello! Connect to a controller to get started.",
            foreground=self.color_scheme['secondary_normal'],
            width=10,
            height=3,
            font=('Arial',12))
        self.status_bar.pack(expand=False, fill="x", padx=5, pady=5)

        self.profile_var = tk.StringVar()
        self.profile_option_switcher = OptionSwitcher(self.root,self.profile_var,None,self.test_profile_options)
        self.profile_option_switcher.pack(expand=False, fill="x", padx=5, pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_window_close)
        self.root.mainloop()


    def del_profile_editor(self):
        try:
            self.cmd_btns['cmd_frame'].destroy()
            self.cmd_btns['txt_frame'].destroy()
        except:
            pass

    def init_profile_editor(self):
        init_profile_editor


    

    def on_window_close(self):
        try:
            if tk.messagebox.askokcancel("Quit", "Do you want to quit?"):
                self.shutdown()
                self.root.destroy()
        except AttributeError:
            self.shutdown()
            self.root.destroy()


    def shutdown(self):
        try:
            pass
        except:
            raise



class ProfileHandler:
    def __init__(file_types, curr_config_file):
        self.file_types  = file_types
        self.curr_config_file  = curr_config_file
        

    def open_file(self):
        filepath = fdialog.askopenfilename(
            filetypes=self.file_types,
            initialdir=self.curr_config_file['dirname'],
            initialfile=self.curr_config_file['basename']
        )
        if not filepath:
            return None
        self.curr_config_file['basename'] = os.path.basename(filepath)
        self.curr_config_file['dirname'] = os.path.dirname(filepath)
        self.load_file()

        return self.curr_config_file


    def save_file_as(self):
        filepath = fdialog.asksaveasfilename(
            defaultextension="txt",
            filetypes=self.file_types,
            initialdir=self.curr_config_file['dirname'],
            initialfile=self.curr_config_file['basename']
        )
        if not filepath:
            return None

        self.curr_config_file['basename'] = os.path.basename(filepath)
        self.curr_config_file['dirname'] = os.path.dirname(filepath)

        return self.curr_config_file

    def save_config_file(self):
        """Save the current file as a new file."""
        filepath = os.path.join(
            self.curr_config_file['dirname'],
            self.curr_config_file['basename']
            )
        
        with open(filepath, "w") as output_file:
            text = self.txt_edit.get(1.0, tk.END)
            output_file.write(text)
        
        
        self.root.title(f"Pressure Control Interface | Config Editor - {os.path.basename(filepath)}")
        self.load_file()


    def load_file(self):
        """Open a file for editing."""
        if self.curr_config_file['dirname'] is None or self.curr_config_file['basename'] is None:
            return

        filepath = os.path.join(
            self.curr_config_file['dirname'],
            self.curr_config_file['basename']
            )
        
        new_config = False
        if filepath.endswith(".yaml"):
            new_config = self.get_config(filepath)
        if new_config:

    def get_config(self, filename):
        try:
            config = load_yaml(filename)
            if isinstance(config, dict):
                if config.get('channels'):
                    basename = os.path.basename(filename)
                    return config
                else:
                    print('Incorrect config format')
                    return False
            else:
                print('Incorrect config format')
                return False
            
        except:
            print('New config was not loaded')
            return False



if __name__ == "__main__":
    gui = ArmstronControlGui()
    try:
        if len(sys.argv)==2:
            gui.load_settings(sys.argv[1])
        gui.init_gui()
    except KeyboardInterrupt:
        gui.on_window_close()