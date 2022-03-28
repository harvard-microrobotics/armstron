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

from armstron.gui.utils import Spinbox, OptionSwitcher
import armstron.utils as utils

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
        

    def open_file(self, direct=False):

        self.callbacks['open_before']()

        if not direct:
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

        from sys import platform
        if platform == "linux" or platform == "linux2":
            cmd='xdg-open'
        elif platform == "darwin":
            cmd='open'
        
        elif 'win' in platform:
            cmd='start'

        os.system(r'%s %s'%(cmd,self.curr_config_file['dirname']))
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