#!/usr/bin/env python
# tkinter Stuff
import os
import sys
import copy
import time
import threading
from turtle import width
if sys.version_info[0] == 3:
    import tkinter as tk
    import tkinter.ttk as ttk
    import tkinter.filedialog as fdialog
else:
    import Tkinter as tk
    import ttk
    import tkFileDialog as fdialog

from ttkthemes import ThemedTk

# ROS Stuff
try:
    import armstron.utils as utils
    import rospy
    import rospkg
    import actionlib
    import json
    import armstron.msg as msg
    from armstron.srv import Balance, Estop
    from armstron.test_interface import TestRunner
    from armstron.gui.profile_editor import ProfileEditor
    from armstron.gui.profile_handler import ProfileHandler
    from armstron.gui.utils import Spinbox, OptionSwitcher
    filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')
except:
    raise
    # Don't load ros stuff (for development on a machine without ROS)
    filepath_config = '../config'
    sys.path.append("../src/armstron")
    import utils 




class ArmstronControlGui:
    def __init__(self):
        self.config_folder=filepath_config
        self.load_settings()
        self.default_option = "< Choose >"
        self.curr_profile_file = {'basename':None, 
                                  'dirname':os.path.join(self.config_folder, 'test_profiles')}

        self.test_handler = TestRunner('armstron')


    def load_settings(self, filename="default.yaml"):
        self.settings = utils.load_yaml(os.path.join(self.config_folder,'gui',filename))

        self.test_profile_path = os.path.join(self.config_folder,'test_profiles')
        self.test_profile_options = [f for f in os.listdir(self.test_profile_path) if os.path.isfile(os.path.join(self.test_profile_path, f))]
        self.test_profile=None

        self.file_types = self.settings['config_file_types']
        self.data_file_types = self.settings['data_file_types']
        self.color_scheme = self.settings['color_scheme']
        self.curr_save_file = {'basename':'test.csv',
                               'dirname': os.path.expanduser(self.settings['data_save_loc'])}
        
        if not os.path.exists(self.curr_save_file['dirname']):
            os.makedirs(self.curr_save_file['dirname'])


    def update_save_file(self):
        self.curr_save_file = self.save_handler.curr_config_file
        self.test_handler.set_savefile(os.path.join(self.curr_save_file['dirname'],self.curr_save_file['basename']))


    def update_config(self):
        try:
            self.curr_profile_file = self.profile_handler.curr_config_file
            self.status_bar.configure(text="Profile: "+str(self.curr_profile_file['basename']))
            self.test_profile = self.profile_handler.get_config()
            if self.test_profile is not None:
                self.update_profile_editor()
                self.test_handler.set_profile(self.test_profile)
                self._enable_testing()

            else:
                self._disable_testing()
        except:
            raise


    def get_config_from_gui(self):
        self.test_profile = self.profile_editor.get_values()
        if self.test_profile is not None:
            self.profile_handler.set_config(self.test_profile)
            self.test_handler.set_profile(self.test_profile)


    def run_test(self):
        self.get_config_from_gui()
        self.test_handler.run_test(wait_for_finish=False)


    def init_gui(self):
        # Make a new window
        self.root = ThemedTk(theme="plastik")#tk.Tk()
        self.root.title("Armstron Test Interface")

        # Add the statusbar
        self.status_bar = tk.Label(self.root, text="Hello! Load a profile to get started.",
            foreground=self.color_scheme['primary_normal'],
            width=50,
            height=3,
            font=('Arial',12, 'bold'))
        self.status_bar.pack(expand=False, fill="x", padx=5, pady=5)

        self.init_test_buttons()

        fr_btns= tk.Frame(self.root, bd=2)
        fr_btns.pack(expand=False, fill="x", padx=5, pady=5)
        self.init_profile_handler(fr_btns)

        #self.profile_var = tk.StringVar()
        #self.profile_option_switcher = OptionSwitcher(self.root,self.profile_var,None,self.test_profile_options)
        #self.profile_option_switcher.pack(expand=False, fill="x", padx=5, pady=5)


        self.root.protocol("WM_DELETE_WINDOW", self.on_window_close)
        self.root.mainloop()


    def _enable_testing(self):
        self.test_buttons['run'].configure(state='normal')
    

    def _disable_testing(self):
        self.test_buttons['run'].configure(state='disabled')


    def init_test_buttons(self):

        test_fr = tk.Frame(self.root, bd=2)
        test_fr.pack(expand=False, fill="x", side='top', padx=5, pady=5)

        bal_fr = tk.LabelFrame(test_fr, text="Balance")

        bal_pose_btn = ttk.Button(bal_fr,
            text = 'Balance Pose',
            command=lambda : self.test_handler.balance('pose'),
            state='normal')

        bal_ft_btn = ttk.Button(bal_fr,
            text = 'Balance F/T',
            command=lambda : self.test_handler.balance('ft'),
            state='normal')

        bal_pose_btn.pack(expand=False, fill="x")
        bal_ft_btn.pack(expand=False, fill="x")

        bal_fr.pack(expand=False,side='left')

        run_fr = tk.LabelFrame(test_fr, text="Run")

        start_btn = tk.Button(run_fr,
            text = 'Run Test',
            font=('Arial', 16, "bold"),
            command=self.run_test,
            state='disabled')

        stop_btn = tk.Button(run_fr,
            text = 'STOP',
            font=('Arial', 16, "bold"),
            fg='#FF0000',
            command=self.test_handler.estop,
            state='normal')

        start_btn.pack(expand=True, fill="x")
        stop_btn.pack(expand=True, fill="x")

        run_fr.pack(expand=False, side='right')

        self.test_buttons={
            'balance_pose': bal_pose_btn,
            'balance_ft': bal_ft_btn,
            'run': start_btn,
            'estop': stop_btn,
        }


    def del_profile_handler(self):
        try:
            del self.profile_handler
            del self.save_handler
        except:
            pass


    def init_profile_handler(self, parent):
        self.del_profile_handler()

        self.profile_handler = ProfileHandler(
            parent,
            self.file_types,
            self.curr_profile_file,
            incldue_btns = ['open', 'saveas'],
            name="Test Profile"
            )
        self.profile_handler.set_callback('open_after',self.update_config)
        self.profile_handler.set_callback('saveas_before',self.get_config_from_gui)
        self.profile_handler.set_callback('saveas_after',lambda : self.profile_handler.open_file(direct=True))


        self.save_handler = ProfileHandler(
            parent,
            self.data_file_types,
            self.curr_save_file,
            incldue_btns = ['saveas', 'folder'],
            name="Save Data",
            side='right'
            )
        self.save_handler.buttons['saveas'].configure(text="Set File", state='normal')
        self.save_handler.buttons['folder'].configure(text="Open Folder", state='normal')

        self.save_handler.set_callback('saveas_after',self.update_save_file)


    def update_profile_editor(self):
        try:
            self.profile_editor.update_inputs(self.test_profile)
        except:     
            # create a profile editor
            self.profile_editor = ProfileEditor(self.root,
                                    self.test_profile,
                                    self.settings,
                                    self.root,
                                    colors=self.color_scheme['stop_conditions'],)

    
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
            self.test_handler.shutdown()
        except:
            raise




if __name__ == "__main__":
    rospy.init_node('v_inst_test_gui', disable_signals=True)
    gui = ArmstronControlGui()
    try:
        if len(sys.argv)==2:
            gui.load_settings(sys.argv[1])
        gui.init_gui()
    except KeyboardInterrupt:
        gui.on_window_close()