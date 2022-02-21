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

        self.file_types = self.settings['config_file_types']
        self.data_file_types = self.settings['data_file_types']
        self.color_scheme = self.settings['color_scheme']


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


    def del_profile_editor(self):
        pass

    def update_profile_editor(self):
        self.del_profile_editor()
        
        # create a profile editor
        self.profile_editor = ProfileEditor(self.root, self.test_profile, self.settings)

    
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
    def __init__(self, parent, file_types, curr_config_file, incldue_btns=['open', 'save','saveas'], name="", side='left'):
        file_types_tup=[]
        for ft in file_types:
            file_types_tup.append(tuple(ft))
        self.file_types  = file_types_tup
        self.curr_config_file  = curr_config_file
        self.config=None
        self._init_buttons(parent, incldue_btns, name, side=side)
        self.callbacks={'open_before':self._empty, 'open_after':self._empty,
                        'save_before':self._empty, 'save_after':self._empty, 
                        'saveas_before':self._empty, 'saveas_after':self._empty,
                        'folder_before':self._empty, 'folder_after':self._empty}

    def _empty(self):
        pass


    def set_callback(self, btn_name, cb):
        """
        Set a button callback by name

        Parameters
        ----------
        btn_name : str
            The button name to attach the callback to
        cb : function
            The callback function to attach
        """
        self.callbacks[btn_name]=cb
    

    def _init_buttons(self, parent, incldue_btns, name, side='left'):
        default_btns = ['open', 'save','saveas']
        self.buttons={}
        for btn in default_btns:
            self.buttons[btn] = None

        self.fr_buttons = tk.LabelFrame(parent, bd=2, text=name)

        if 'open' in incldue_btns:
            open_btn = ttk.Button(self.fr_buttons,
                text="Open",
                command = self.open_file
                )
            open_btn.grid(row=1, column=0, sticky='ns', padx=5, pady=5)
            self.buttons["open"] = open_btn

        if 'folder' in incldue_btns:
            folder_btn = ttk.Button(self.fr_buttons,
                text="Open Folder",
                command = self.open_folder
                )
            folder_btn.grid(row=1, column=0, sticky='ns', padx=5, pady=5)
            self.buttons["folder"] = folder_btn

        if 'save' in incldue_btns:
            save_btn = ttk.Button(self.fr_buttons,
                text="Save",
                command = self.save_file,
                state = 'disabled',
                )
            save_btn.grid(row=1, column=1, sticky='ns', padx=5, pady=5)
            self.buttons["save"] = save_btn

        if 'saveas' in incldue_btns:
            save_as_btn = ttk.Button(self.fr_buttons,
                text="Save As",
                command = self.save_file_as,
                state = 'disabled',
                )
            save_as_btn.grid(row=1, column=2, sticky='ns', padx=5, pady=5)
            self.buttons["saveas"] = save_as_btn

        
        self.fr_buttons.pack(expand=False, fill="y", side=side)

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

        self.callbacks['open_before']()

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

        self.callbacks['open_after']()

        return self.curr_config_file


    def save_file_as(self):

        self.callbacks['saveas_before']()

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

        self.callbacks['saveas_after']()

        return self.curr_config_file


    def save_file(self):
        self.callbacks['save_before']()

        if self.config is not None:
            """Save the current file as a new file."""
            filepath = os.path.join(
                self.curr_config_file['dirname'],
                self.curr_config_file['basename']
                )

            if not os.path.exists(self.curr_config_file['dirname']):
                os.makedirs(self.curr_config_file['dirname'])
            
            with open(filepath, "w") as output_file:
                utils.save_yaml(self.config, filepath)

        self.callbacks['save_after']()
        

    def open_folder(self):
        self.callbacks['folder_before']()
        os.system(r'xdg-open %s'%(self.curr_config_file['dirname']))
        self.callbacks['folder_after']()


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
    def __init__(self, parent, profile, default_values):
        self.variable_tree, self.profile = self._generate_variable_tree(profile)
        self.stop_values = default_values['stop_conditions']
        self.balance_values = default_values['balance_options']

        self._init_inputs(parent, self.profile, self.variable_tree)

    def _empty(self):
        pass


    def _split_condition_str(self, string):
        """
        Split a stop condition string key into a dictionary

        Parameters
        ----------
        string : str
            String key for the condition
        
        Returns
        -------
        out_dict : dict
            Condition disctionary with ``condition`` and ``signal`` values.
        """
        out = {'condition':None, 'signal': None}
        
        string_list = string.split('_',1)
        if len(string_list)==2:
            out['condition'] = string_list[0]
            out['signal'] = string_list[1]
        else:
            out['signal'] = string_list[0]

        return out 

    
    def _combine_condition(self,condition):
        """
        Combine a stop condition dictionary to recover the string key

        Parameters
        ----------
        condition : dict
            Condition to combine
        
        Returns
        -------
        out_str : str
            String key for the condition
        """
        out_str=""
        if condition['condition'] is not None:
            out_str+=condition['condition']
            out_str+="_"
        if condition['signal'] is not None:
            out_str+=condition['signal']
        return out_str


    def _generate_variable_tree(self, profile):
        """
        Generate a tree of variables for a given profile, first
        expanding the stop conditions to enable editing of conditions.

        Parameters
        ----------
        profile : dict
            A test profile to convert
        
        Returns
        -------
        var_tree : dict
            A tree of Tk variables with the same structure as the profile
        profile_expanded : dict
            A copy of the test profile with the stop conditions expanded. 
        """
        profile_expanded = copy.deepcopy(profile)

        params = profile_expanded['params']

        # Make each group into a list of steps
        for key in params:
            curr_group = params[key]
            if isinstance(curr_group, dict):
                params[key] = [curr_group]

        # Expand stop conditions
        for key in params:
            curr_group = params[key]
            for curr_step in curr_group:
                stop_conditions = curr_step.get('stop_conditions',None)
                if stop_conditions is not None:
                    condition_list = []
                    for key in stop_conditions:
                        cond = self._split_condition_str(key)
                        condition_list.append({'condition':cond['condition'],'signal':cond['signal'], 'value':stop_conditions[key]})

                    curr_step['stop_conditions'] = condition_list

        # Generate Tk variables
        var_tree = self._generate_tk_variables(profile_expanded)
        return var_tree, profile_expanded


    def _generate_tk_variables(self,input_obj):
        """
        Generate a tree of Tk variables that matches the input structure

        Parameters
        ----------
        input_obj : Any()
            Input object to convert
        
        Returns
        -------
        output_obj: Any()
            A tree of Tk variables with the same structure as the input
        """
        var = None
        if isinstance(input_obj, str):
            var = tk.StringVar()
        elif isinstance(input_obj, int):
            var = tk.DoubleVar()
        elif isinstance(input_obj, float):
            var = tk.DoubleVar()
        elif isinstance(input_obj, bool):
            var =  tk.BooleanVar()

        if var is not None:
            var.set(input_obj)
            return var

        if isinstance(input_obj, list):
            var_list = []
            for var in input_obj:
                var_list.append(self._generate_tk_variables(var))
            return var_list
        
        elif isinstance(input_obj, dict):
            var_tree={}

            for key in input_obj:
                curr_value = input_obj[key]
                var_tree[key] = self._generate_tk_variables(curr_value)
 
            return var_tree
        else:
            return None
        

    def _get_tk_values(self, input_obj):
        """
        Get values of tk variables recursively

        Parameters
        ----------
        input_obj : Any()
            Input object to convert
        
        Returns
        -------
        output_obj: Any()
            A tree of numbers with the same structure as the input
        """
        if isinstance(input_obj, tk.StringVar) or \
            isinstance(input_obj, tk.DoubleVar) or \
            isinstance(input_obj, tk.BooleanVar): 
            return input_obj.get()    

        elif isinstance(input_obj, list):
            var_list = []
            for var in input_obj:
                var_list.append(self._get_tk_values(var))
            return var_list
        
        elif isinstance(input_obj, dict):
            var_tree={}

            for key in input_obj:
                curr_value = input_obj[key]
                var_tree[key] = self._get_tk_values(curr_value)
 
            return var_tree
        else:
            return None


    def get_values(self):
        """
        Get the values set by the gui

        Returns
        -------
        profile_vals : dict
            A dictionary of new profile values set by the gui
        """
        profile = self._get_tk_values(self.variable_tree)

        params = profile['params']

        # Condense stop conditions back into a list
        for key in params:
            curr_group = params[key]
            for curr_step in curr_group:
                stop_conditions = curr_step.get('stop_conditions',None)
                if stop_conditions is not None:
                    condition_dict = {}
                    for values in stop_conditions:
                        cond_str = self._combine_condition(values)
                        condition_dict[cond_str] = values['value']

                    curr_step['stop_conditions'] = condition_dict
        
        return profile


    def _make_input_group(self, parent, config, vars, index):
        fr_group = tk.LabelFrame(parent, text="Step %d"%(index), font=('Arial', 10, 'bold'), bd=2)

        balance = config.get('balance', False)
        if balance:
            label = tk.Label(fr_group, text="Balance: ")
            label.grid(row=0,column=0, sticky="ew")

            cond = OptionSwitcher(fr_group, vars['balance'], balance, self.balance_values)
            cond.grid(row=0, column=1, sticky='ew')
            return fr_group


        motion = config['motion']
        fr_motion = tk.Frame(fr_group, bd=2)
        

        label = tk.Label(fr_motion, text="Linear: ")
        label.grid(row=0,column=0, sticky="ew")
        idx=1
        for curr, var in zip(motion['linear'], vars['motion']['linear']):
            box = Spinbox(fr_motion, width=7, textvariable=var)
            box.set(curr)
            box.grid(row=0, column=idx, sticky='ew')
            idx+=1

        label = tk.Label(fr_motion,text="Angular: ")
        label.grid(row=1,column=0, sticky="ew")
        idx=1
        for curr, var in zip(motion['angular'], vars['motion']['angular']):
            box = Spinbox(fr_motion,  width=7, textvariable=var)
            box.set(curr)
            box.grid(row=1, column=idx,sticky='ew')
            idx+=1

        fr_motion.pack(expand=True, fill="x")

        stop_conditions = config['stop_conditions']
        fr_stop = tk.Frame(fr_group, bd=2)

        for condition, var in zip(stop_conditions,vars['stop_conditions']):
            fr_stop_inner = tk.Frame(fr_stop)

            cond = OptionSwitcher(fr_stop_inner, var['condition'],
                                    condition['condition'],
                                    self.stop_values[condition['signal']])

            signal = OptionSwitcher(fr_stop_inner,
                                    var['signal'],
                                    condition['signal'],
                                    sorted(self.stop_values.keys()))

            box = Spinbox(fr_stop_inner, textvariable=var['value'])
            box.set(condition['value'])
            cond.pack(expand=False, fill="y", side='left')
            signal.pack(expand=True, fill="y", side='left')
            box.pack(expand=False, fill="y", side='left')
            fr_stop_inner.pack(expand=False,fill='y')

        fr_stop.pack(expand=True, fill="x")

        return fr_group
        
    

    def _init_inputs(self, parent, profile, var_tree):
        self.fr_buttons = tk.Frame(parent, bd=2)

        preload = profile['params'].get('preload')
        test = profile['params'].get('test')
        preload_vars = var_tree['params'].get('preload')
        test_vars = var_tree['params'].get('test')

        fr_preload = tk.LabelFrame(self.fr_buttons, text="Preload", font=('Arial', 12, 'bold'), bd=2)
        for idx,seg in enumerate(preload):
            fr = self._make_input_group(fr_preload,seg,preload_vars[idx], idx)
            fr.pack(expand=False, fill="both", padx=5, pady=5, side='top')

        fr_preload.pack(expand=False, fill="both", padx=5, pady=5, side='left')


        fr_test = tk.LabelFrame(self.fr_buttons ,text="Main Test", font=('Arial', 12, 'bold'), bd=2)
        for idx,seg in enumerate(test):
            fr = self._make_input_group(fr_test,seg, test_vars[idx], idx)
            fr.pack(expand=False, fill="both", padx=5, pady=5, side='top')

        fr_test.pack(expand=False, fill="both", padx=5, pady=5, side='left')
        
        self.fr_buttons.pack(expand=False, fill="x", side='top')


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