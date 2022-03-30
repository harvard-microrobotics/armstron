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

from armstron.gui.utils import Spinbox, OptionSwitcher, ScrollbarLabelFrame


class ProfileEditor:
    def __init__(self,
            parent,
            profile,
            default_values,
            root,
            colors=None):
        self.variable_tree, self.profile = self._generate_variable_tree(profile)
        self.stop_values = default_values['stop_conditions']
        self.balance_values = default_values['balance_options']

        self.parent = parent
        self.root = root

        if colors is None:
            self.colors = {}
            for key in self.stop_values:
                self.colors[key] = ['#ffffff', '#ffffff']
        else:
            self.colors = colors

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
            #cb = lambda name, index, val: self.update_inputs()
            #var.trace('w', cb)
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
        
        return copy.deepcopy(profile)


    def _make_input_group(self, parent, config, vars, index):
        # Get the step type and values
        step_types = ['jog','pose','balance']
        for step_type in step_types:
            if config.get(step_type,None) is not None:
                break
            else:
                step_type=None
        if step_type is None:
            raise ValueError("No valid step types were detected...")

        step_vals = config[step_type]

        # Create a widget group
        fr_group = tk.LabelFrame(parent, text="Step %d"%(index),
            font=('Arial', 10, 'bold'), bd=2,
            fg = self.colors['default'][0])

        # Make balance inputs
        if step_type == 'balance':
            label = tk.Label(fr_group, text="Balance: ", bg=self.colors['default'][1])
            label.grid(row=0,column=0, sticky="e")

            cond = OptionSwitcher(fr_group, vars['balance'], step_vals, self.balance_values)
            cond.grid(row=0, column=1, sticky='ew')
            fr_group.configure(text="Step %d: %s"%(index, 'Balance'))

        # Make pose inputs
        if step_type == 'pose':
            fr_pose = tk.Frame(fr_group, bd=2)
        

            label = tk.Label(fr_pose, text="Time: ")
            label.grid(row=0,column=0, sticky="e")
            box = Spinbox(fr_pose, width=7, textvariable=vars[step_type]['time'])
            box.set(step_vals['time'])
            box.grid(row=0, column=1, sticky='ew')

            label = tk.Label(fr_pose, text="Position: ")
            label.grid(row=1,column=0, sticky="e")
            idx=1
            for curr, var in zip(step_vals['position'], vars[step_type]['position']):
                box = Spinbox(fr_pose, width=7, textvariable=var)
                box.set(curr)
                box.grid(row=1, column=idx, sticky='ew')
                idx+=1

            label = tk.Label(fr_pose,text="Orientation: ")
            label.grid(row=2,column=0, sticky="e")
            idx=1
            for curr, var in zip(step_vals['orientation'], vars[step_type]['orientation']):
                box = Spinbox(fr_pose,  width=7, textvariable=var)
                box.set(curr)
                box.grid(row=2, column=idx,sticky='ew')
                idx+=1

            fr_pose.pack(expand=True, fill="x")
            fr_group.configure(text="Step %d: %s"%(index, 'Pose'))

        # Make jog inputs
        if step_type == 'jog':
            fr_motion = tk.Frame(fr_group, bd=2)
            

            label = tk.Label(fr_motion, text="Linear: ")
            label.grid(row=0,column=0, sticky="e")
            idx=1
            for curr, var in zip(step_vals['linear'], vars[step_type]['linear']):
                box = Spinbox(fr_motion, width=7, textvariable=var)
                box.set(curr)
                box.grid(row=0, column=idx, sticky='ew')
                idx+=1

            label = tk.Label(fr_motion,text="Angular: ")
            label.grid(row=1,column=0, sticky="e")
            idx=1
            for curr, var in zip(step_vals['angular'], vars[step_type]['angular']):
                box = Spinbox(fr_motion,  width=7, textvariable=var)
                box.set(curr)
                box.grid(row=1, column=idx,sticky='ew')
                idx+=1

            fr_motion.pack(expand=True, fill="x")
            fr_group.configure(text="Step %d: %s"%(index, 'Jog'))

        # Get stop conditions and add them if they exist
        stop_conditions = config.get('stop_conditions', False)
        if stop_conditions:
            fr_stop = tk.Frame(fr_group, bd=2)

            for condition, var in zip(stop_conditions,vars['stop_conditions']):
                fr_stop_inner = tk.Frame(fr_stop, bg=self.colors[condition['signal']][1], bd=2)

                cond = OptionSwitcher(fr_stop_inner, var['condition'],
                                        condition['condition'],
                                        self.stop_values[condition['signal']])

                signal = OptionSwitcher(fr_stop_inner,
                                        var['signal'],
                                        condition['signal'],
                                        sorted(self.stop_values.keys()))

                def cb (*args):
                    cond.update_options(self.stop_values[var['signal'].get()])
                    fr_stop_inner.configure(bg=self.colors[var['signal'].get()][1])
                    
                var['signal'].trace("w", cb)
                # cond.update_options(self.stop_values[value])

                box = Spinbox(fr_stop_inner, textvariable=var['value'])
                box.set(condition['value'])
                cond.pack(expand=False, fill="y", side='left')
                signal.pack(expand=True, fill="y", side='left')
                box.pack(expand=False, fill="y", side='left')
                fr_stop_inner.pack(expand=False,fill='y')

            fr_stop.pack(expand=True, fill="x")

        return fr_group

    
    def _move_step(self, key, idx, dir):
        steps = self.profile['params'][key]
        if dir =='up':
            if idx == 0:
                return
            else:
                idx_mv = idx-1
        else: #down
            if idx == len(steps)-1:
                return
            else:
                idx_mv = idx+1

        v = self.variable_tree['params'][key]
        v.insert(idx_mv, v.pop(idx))
        p = self.profile['params'][key]
        p.insert(idx_mv, p.pop(idx))

        # Regenerate 
        self.update_inputs()


    def _del_step(self, key, idx):
        steps = self.profile['params'][key]
        v = self.variable_tree['params'][key]
        v.pop(idx)
        p = self.profile['params'][key]
        p.pop(idx)

        # Regenerate 
        self.update_inputs()


    def _add_step(self, key, type):
        if type == 'pose':
            new_step = {'pose':{'position':[0,0,0],'orientation':[0,0,0]}}
        elif type == 'balance':
            new_step = {'balance':'pose'}
        elif type == 'jog':
            new_step = {'jog':{'linear':[0,0,0],'angular':[0,0,0]},
                        'stop_conditions': {'max_time': 5}}

        profile = self.get_values()
        p = profile['params'][key]
        p.append(new_step)

        self.variable_tree, self.profile = self._generate_variable_tree(profile)

        # Regenerate 
        self.update_inputs()

        

    def _make_controls(self, parent, key, idx):
        fr_group = tk.Frame(parent, bd=2)

        up_btn = tk.Button(fr_group,
                text=u'\u25B2',
                command = lambda key=key, idx=idx : self._move_step(key, idx, 'up'),
                state = 'normal',)
        dn_btn = tk.Button(fr_group,
                text=u'\u25BC',
                command = lambda key=key, idx=idx : self._move_step(key, idx, 'down'),
                state = 'normal',)
        del_btn = tk.Button(fr_group,
                text=u'\u274C',
                command = lambda key=key, idx=idx : self._del_step(key, idx),
                state = 'normal',)

        del_btn.pack(expand=False, fill="x", padx=1, pady=0, side='top')
        up_btn.pack(expand=False, fill="x", padx=1, pady=0, side='top')
        dn_btn.pack(expand=False, fill="x", padx=1, pady=0, side='top')       

        return fr_group


    def _make_add_button(self, parent, key):
        # Create a widget group
        fr_group = tk.LabelFrame(parent, text="Add Step",
            font=('Arial', 10, 'bold'), bd=2,
            fg = self.colors['default'][0])

        add_balance = ttk.Button(fr_group,
                text="Balance",
                command = lambda key=key, : self._add_step(key, 'balance'),
                state = 'normal',)

        add_pose = ttk.Button(fr_group,
                text="Pose",
                command = lambda key=key, : self._add_step(key, 'pose'),
                state = 'normal')

        add_jog = ttk.Button(fr_group,
                text="Jog",
                command = lambda key=key, : self._add_step(key, 'jog'),
                state = 'normal')
        

        add_balance.grid(row=0,column=0, sticky="ew")
        add_pose.grid(row=0,column=1, sticky="ew")
        add_jog.grid(row=0,column=2, sticky="ew")

        return fr_group


    def clear(self, profile=None):
        if profile is None:
            profile = self.get_values()
        self._del_inputs()
        self.variable_tree, self.profile = self._generate_variable_tree(profile)


    def _del_inputs(self):
        try:
            self.fr_buttons.destroy()        
        except:
            pass


    def update_inputs(self, profile=None):
        self.clear(profile)
        self._init_inputs(self.parent, self.profile, self.variable_tree)


    def _init_inputs(self, parent, profile, var_tree):
        self.fr_buttons = tk.Frame(parent, bd=2)

        preload = profile['params'].get('preload')
        test = profile['params'].get('test')
        preload_vars = var_tree['params'].get('preload')
        test_vars = var_tree['params'].get('test')

        sbf = ScrollbarLabelFrame(self.fr_buttons, text="Preload", font=('Arial', 12, 'bold'), bd=2)
        sbf.pack(expand=True, fill="both", padx=5, pady=5, side='left')
        #canvas = Scrollable(self.fr_buttons)

        fr_preload = sbf.scrolled_frame #tk.LabelFrame(sbf.scrolled_frame, text="Preload", font=('Arial', 12, 'bold'), bd=2)
        for idx,seg in enumerate(preload):
            fr_step=tk.Frame(fr_preload)
            fr_step.pack(expand=False, fill='both', side='top')

            fr_ctrl = self._make_controls(fr_step,'preload',idx)
            fr = self._make_input_group(fr_step,seg,preload_vars[idx], idx)
            fr_ctrl.pack(expand=False, fill="both", padx=5, pady=5, side='left')
            fr.pack(expand=False, fill="both", padx=5, pady=5, side='left')

        fr_step=tk.Frame(fr_preload)
        fr_step.pack(expand=False, fill='both', side='top')

        fr_ctrl = tk.Frame(fr_step, width=45)
        fr = self._make_add_button(fr_step, 'preload', )
        fr_ctrl.pack(expand=False, fill="both", padx=5, pady=5, side='left')
        fr.pack(expand=False, fill="both", padx=5, pady=5, side='left')

        #fr_preload.pack(expand=False, fill="both", padx=5, pady=5, side='left')

        sbf2 = ScrollbarLabelFrame(self.fr_buttons, text="Test", font=('Arial', 12, 'bold'), bd=2)
        sbf2.pack(expand=True, fill="both", padx=5, pady=5, side='left')
        #canvas = Scrollable(self.fr_buttons)

        fr_test = sbf2.scrolled_frame 

        #fr_test = tk.LabelFrame(self.fr_buttons ,text="Main Test", font=('Arial', 12, 'bold'), bd=2)
        for idx,seg in enumerate(test):
            fr_step=tk.Frame(fr_test)
            fr_step.pack(expand=False, fill='both', side='top')

            fr_ctrl = self._make_controls(fr_step, 'test',idx)
            fr = self._make_input_group(fr_step,seg, test_vars[idx], idx)
            fr_ctrl.pack(expand=False, fill="both", padx=5, pady=5, side='left')
            fr.pack(expand=False, fill="both", padx=5, pady=5, side='left')

        fr_step=tk.Frame(fr_test)
        fr_step.pack(expand=False, fill='both', side='top')

        fr_ctrl = tk.Frame(fr_step, width=45)
        fr = self._make_add_button(fr_step, 'test', )
        fr_ctrl.pack(expand=False, fill="both", padx=5, pady=5, side='left')
        fr.pack(expand=False, fill="both", padx=5, pady=5, side='left')

        #fr_test.pack(expand=False, fill="both", padx=5, pady=5, side='left')
        
        self.fr_buttons.pack(expand=True, fill="both", side='top')


    def __del__(self):
        self.fr_buttons.destroy()