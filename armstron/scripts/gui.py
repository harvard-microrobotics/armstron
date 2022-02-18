#!/usr/bin/env python
# tkinter Stuff
import os
import sys
import copy
import time
import threading
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
    filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')
    filepath_data = os.path.join(rospkg.RosPack().get_path('armstron'), 'data')
except:
    # Don't load ros stuff (for development on a machine without ROS)
    filepath_config = '../config'
    filepath_data = '../data'
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
        self.curr_profile_file = {'basename':None, 
                                  'dirname':os.path.join(self.config_folder, 'test_profiles')}

        self.curr_save_file = {'basename':'test.csv', 
                                  'dirname':filepath_data}

        self.test_handler = TestRunner('armstron')


    def load_settings(self, filename="default.yaml"):
        self.settings = utils.load_yaml(os.path.join(self.config_folder,'gui',filename))

        self.test_profile_path = os.path.join(self.config_folder,'test_profiles')
        self.test_profile_options = [f for f in os.listdir(self.test_profile_path) if os.path.isfile(os.path.join(self.test_profile_path, f))]
        self.test_profile=None

        self.file_types = self.settings['file_types']
        self.color_scheme = self.settings['color_scheme']


    def update_config(self):
        try:
            self.test_profile = self.profile_handler.get_config()
            if self.test_profile is not None:
                self.update_profile_editor()
                self.test_handler.set_profile(self.test_profile)
                self.test_handler.set_savefile(os.path.join(self.curr_save_file['dirname'],self.curr_save_file['basename']))
                self._enable_testing()

            else:
                self._disable_testing()

            print(self.test_profile)
        except:
            raise


    def init_gui(self):
        # Make a new window
        self.root = ThemedTk(theme="plastik")#tk.Tk()
        self.root.title("Pressure Control Interface")

        # Add the statusbar
        self.status_bar = tk.Label(self.root, text="Hello! Load a profile to get started.",
            foreground=self.color_scheme['secondary_normal'],
            width=10,
            height=3,
            font=('Arial',12))
        self.status_bar.pack(expand=False, fill="x", padx=5, pady=5)

        self.init_test_buttons()

        self.init_profile_handler()

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
        test_fr.pack(expand=False, fill="x")

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

        start_btn = ttk.Button(run_fr,
            text = 'Run Test',
            command=self.test_handler.run_test,
            state='disabled')

        stop_btn = ttk.Button(run_fr,
            text = 'STOP',
            command=self.test_handler.estop,
            state='normal')

        start_btn.pack(expand=True, fill="x")
        stop_btn.pack(expand=True, fill="x")

        run_fr.pack(expand=False, side='right')

        test_fr.pack(expand=True, fill="x", padx=5, pady=5)

        self.test_buttons={
            'balance_pose': bal_pose_btn,
            'balance_ft': bal_ft_btn,
            'run': start_btn,
            'estop': stop_btn,
        }


    def del_profile_handler(self):
        try:
            del self.profile_handler
        except:
            pass


    def init_profile_handler(self):
        self.del_profile_handler()
        self.profile_handler = ProfileHandler(
            self.root,
            self.file_types,
            self.curr_profile_file,
            incldue_btns = ['open', 'saveas']
            )
        self.profile_handler.set_callback('open',self.update_config)


    def del_profile_editor(self):
        pass

    def update_profile_editor(self):
        self.del_profile_editor()
        
        # create a profile editor
        self.profile_editor = ProfileEditor(self.root, self.test_profile)

    
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


class ProfileHandler:
    def __init__(self, parent, file_types, curr_config_file, incldue_btns=['open', 'save','saveas']):
        file_types_tup=[]
        for ft in file_types:
            file_types_tup.append(tuple(ft))
        self.file_types  = file_types_tup
        self.curr_config_file  = curr_config_file
        self.config=None
        self._init_buttons(parent, incldue_btns)
        self.callbacks={'open':self._empty, 'save':self._empty, 'saveas':self._empty}

    def _empty(self):
        pass


    def set_callback(self, btn_name, cb):
        self.callbacks[btn_name]=cb
    

    def _init_buttons(self, parent, incldue_btns):
        default_btns = ['open', 'save','saveas']
        self.buttons={}
        for btn in default_btns:
            self.buttons[btn] = None

        self.fr_buttons = tk.Frame(parent, bd=2)

        if 'open' in incldue_btns:
            open_btn = ttk.Button(self.fr_buttons,
                text="Open Profile",
                command = self.open_file
                )
            open_btn.grid(row=1, column=0, sticky='ns', padx=5, pady=5)
            self.buttons["open"] = open_btn

        if 'save' in incldue_btns:
            save_btn = ttk.Button(self.fr_buttons,
                text="Save Profile",
                command = self.save_file,
                state = 'disabled',
                )
            save_btn.grid(row=1, column=1, sticky='ns', padx=5, pady=5)
            self.buttons["save"] = save_btn

        if 'saveas' in incldue_btns:
            save_as_btn = ttk.Button(self.fr_buttons,
                text="Save Profile As",
                command = self.save_file_as,
                state = 'disabled',
                )
            save_as_btn.grid(row=1, column=2, sticky='ns', padx=5, pady=5)
            self.buttons["saveas"] = save_as_btn

        
        self.fr_buttons.pack(expand=False, fill="x")

    def _check_enable_buttons(self):
        if self.config is not None:
            if self.buttons['save'] is not None:
                self.buttons['save'].configure(state='normal')
            if self.buttons['saveas'] is not None:
                self.buttons['saveas'].configure(state='normal')
        else:
            if self.buttons['save'] is not None:
                self.buttons['save'].configure(state='disabled')
            if self.buttons['saveas'] is not None:
                self.buttons['saveas'].configure(state='disabled')
        

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
        self._check_enable_buttons()

        self.callbacks['open']()

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

        self.save_file()

        self.callbacks['saveas']()

        return self.curr_config_file

    def save_file(self):
        """Save the current file as a new file."""
        filepath = os.path.join(
            self.curr_config_file['dirname'],
            self.curr_config_file['basename']
            )

        if not os.path.exists(self.curr_config_file['dirname']):
            os.makedirs(self.curr_config_file['dirname'])
        
        with open(filepath, "w") as output_file:
            utils.save_yaml(self.config, filepath)

        self.callbacks['save']()
        

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
            new_config = self.load_config(filepath)
        if new_config:
            self.config = new_config


    def get_config(self):
        return self.config


    def set_config(self, config):
        self.config = config


    def load_config(self, filename):
        try:
            config = utils.load_yaml(filename)
            if isinstance(config, dict):
                return config

            else:
                print('Incorrect config format')
                return False
        except:
            print('New config was not loaded')
            return False

    def __del__(self):
        self.fr_buttons.destroy()



class ProfileEditor:
    def __init__(self, parent, profile):
        self.profile = profile
        self.stop_values = ['max_time','max_force_x', 'max_force_y', 'max_force_z',
                            'min_force_x', 'min_force_y', 'min_force_z',
                            'max_torque_x', 'max_torque_y', 'max_torque_z',
                            'min_torque_x', 'min_torque_y', 'min_torque_z',
                            'max_position_x', 'max_position_y', 'max_position_z',
                            'min_position_x', 'min_position_y', 'min_position_z']

        self._init_inputs(parent, profile)

    def _empty(self):
        pass


    def set_callback(self, btn_name, cb):
        self.callbacks[btn_name]=cb


    def make_input_group(self, parent, config):
        fr_group = tk.Frame(parent, bd=2)

        motion = config['motion']
        fr_motion = tk.Frame(fr_group, bd=2)
        
        for curr in motion['linear']:
            box = Spinbox(fr_motion)
            box.set(curr)
            box.pack(expand=False, fill="y")

        for curr in motion['angular']:
            box = Spinbox(fr_motion)
            box.set(curr)
            box.pack(expand=False, fill="y")
        

        fr_motion.pack(expand=True, fill="x")

        stop_conditions = config['stop_conditions']
        fr_stop = tk.Frame(fr_group, bd=2)

        for key in stop_conditions:
            fr_stop_inner = tk.Frame(fr_stop)
            var = tk.StringVar()
            label = OptionSwitcher(fr_stop_inner, var, key, self.stop_values)

            box = Spinbox(fr_stop_inner)
            box.set(stop_conditions[key])
            label.pack(expand=False, fill="x")
            box.pack(expand=False, fill="x")
            fr_stop_inner.pack(expand=False,fill='y')

        fr_stop.pack(expand=True, fill="x")

        return fr_group
        
    

    def _init_inputs(self, parent, profile):
        self.fr_buttons = tk.Frame(parent, bd=2)

        preload = profile['params'].get('preload')
        test = profile['params'].get('test')

        if isinstance(preload,dict):
            preload = [preload]

        if isinstance(test,dict):
            test = [test]

        fr_preload = tk.LabelFrame(self.fr_buttons, text="Preload", bd=2)
        for seg in preload:
            fr = self.make_input_group(fr_preload,seg)
            fr.pack(expand=True, fill="x", padx=5, pady=5, side='left')

        fr_preload.pack(expand=True, fill="x", padx=5, pady=5)


        fr_test = tk.LabelFrame(self.fr_buttons ,text="Main Test", bd=2)
        for seg in test:
            fr = self.make_input_group(fr_test,seg)
            fr.pack(expand=True, fill="x", padx=5, pady=5, side='left')

        fr_test.pack(expand=True, fill="x", padx=5, pady=5)
        
        self.fr_buttons.pack(expand=False, fill="x")


    def __del__(self):
        self.fr_buttons.destroy()




if __name__ == "__main__":
    rospy.init_node('v_inst_test_gui', disable_signals=True)
    gui = ArmstronControlGui()
    try:
        if len(sys.argv)==2:
            gui.load_settings(sys.argv[1])
        gui.init_gui()
    except KeyboardInterrupt:
        gui.on_window_close()