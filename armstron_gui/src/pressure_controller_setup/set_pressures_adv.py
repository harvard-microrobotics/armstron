import os
import rospy
import rospkg
import actionlib
import copy
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import qDebug, Qt
from python_qt_binding.QtWidgets import QWidget, QLayout, QVBoxLayout, QHBoxLayout, QSlider, QSpinBox, QDoubleSpinBox, QPushButton, QLabel
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog

import pressure_controller_ros.msg


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('armstron_gui'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Set Pressures')

        self.settings=self.get_settings()
        
        self.init_sliders()

        self._client = actionlib.SimpleActionClient('pressure_control', pressure_controller_ros.msg.CommandAction)
        self._client_connected=self._client.wait_for_server(timeout=rospy.rostime.Duration(1))

        if not self._client_connected:
            print("No command server avaiable... changes will not be sent")

        self.set_graph_state(True)
        self.send_channel_state(0,self.settings['channel_states'][0])


        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)



    def init_sliders(self):
        sliderbox = self._widget.findChild(QLayout,'Sliders')

        firstCol = QVBoxLayout()
        graph_button=QPushButton()
        graph_button.setCheckable(True)
        graph_button.setText("Graph Off")
        graph_button.toggle()
        graph_button.clicked.connect(self.set_graph_state)
        self.graph_button = graph_button

        reset_button=QPushButton()
        reset_button.setCheckable(False)
        reset_button.setText("Reset")
        reset_button.clicked.connect(self.set_reset)

        self.graph_button = graph_button
        self.reset_button = reset_button

        firstCol.addWidget(graph_button)
        firstCol.addWidget(reset_button)


        firstCol.addWidget(graph_button)
        firstCol.setAlignment(graph_button,Qt.AlignVCenter)

        zero_button=QPushButton()
        zero_button.setCheckable(False)
        zero_button.setText("Set All Zero")
        zero_button.clicked.connect(self.set_pressure_zero)
        self.zero_button = zero_button
        firstCol.addWidget(zero_button)
        firstCol.setAlignment(zero_button,Qt.AlignVCenter)



        transition_box = QVBoxLayout()
        label = QLabel()
        label.setAlignment(Qt.AlignCenter)
        label.setText("Transition Time")

        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(0)
        spinbox.setMaximum(10)
        spinbox.setValue(self.settings['transitions'])
        spinbox.setDecimals(1)
        spinbox.setSingleStep(0.1)
        spinbox.setSuffix(" sec")

        spinbox.valueChanged.connect(self.set_transition_value)

        transition_box.addWidget(label)
        transition_box.addWidget(spinbox)
        transition_box.setAlignment(label,Qt.AlignBottom)
        transition_box.setAlignment(spinbox,Qt.AlignTop)
        firstCol.addLayout(transition_box)

        self.sliders = []
        sliderbox.addLayout(firstCol)

        all_rows_layout = QVBoxLayout()

        g_idx = 0

        for row in self.settings['gui_config']:
            num_groups_row = len(row)
            row_layout = QHBoxLayout()
            for gr_idx, s_group in enumerate(row): 
                g_channels = s_group['channels']
                g_layout = s_group['layout']

                if 'horiz' in g_layout:
                    group_layout = QHBoxLayout()
                else:
                    group_layout = QVBoxLayout()

                control_group = {'sliders': [], 'on_off': None}

                label = QLabel()
                label.setText("Group. %d"%(g_idx+1))
                label.setAlignment(Qt.AlignCenter)
                group_layout.addWidget(label)

                for c_idx, s_idx in enumerate(g_channels):
                    idx = s_idx*1
                    
                    slider_group={'slider':None, 'number':None}
                    
                    layout_cluster = QVBoxLayout()

                    labelfirst = QLabel()
                    labelfirst.setAlignment(Qt.AlignCenter)

                    labellast = QLabel()
                    labellast.setAlignment(Qt.AlignCenter)

                    layout = QVBoxLayout()

                    if 'diff' in g_layout and c_idx == 0:

                        sublayout=QHBoxLayout()

                        layout.setAlignment(Qt.AlignVCenter)

                        slider = QSlider(Qt.Horizontal)
                        slider.setMinimum(-100)
                        slider.setMaximum(100)
                        slider.setValue(0)
                        slider.setTickPosition(QSlider.TicksRight)
                        slider.setTickInterval(20)

                        spinbox = QDoubleSpinBox()
                        spinbox.setMinimum(-10)
                        spinbox.setMaximum(10)
                        spinbox.setValue(0)
                        spinbox.setDecimals(1)
                        spinbox.setSingleStep(0.1)

                        labellast.setText("%0.1f"%(10)) # These are flipped becasue of order
                        labelfirst.setText("%0.1f"%(-10))

                        max_label = labellast
                        min_label = labelfirst

                    else:
                        layout.setAlignment(Qt.AlignHCenter)

                        slider = QSlider(Qt.Vertical)
                        slider.setMinimum(self.settings['min_pressure'][idx]*10.0)
                        slider.setMaximum(self.settings['max_pressure'][idx]*10.0)
                        slider.setValue(0)
                        slider.setTickPosition(QSlider.TicksRight)
                        slider.setTickInterval(20)

                        spinbox = QDoubleSpinBox()
                        spinbox.setMinimum(self.settings['min_pressure'][idx])
                        spinbox.setMaximum(self.settings['max_pressure'][idx])
                        spinbox.setValue(0)
                        spinbox.setDecimals(1)
                        spinbox.setSingleStep(0.1)

                        labelfirst.setText("%0.1f"%(self.settings['max_pressure'][idx]))
                        labellast.setText("%0.1f"%(self.settings['min_pressure'][idx]))

                        max_label = labelfirst
                        min_label = labellast

                    
                    cb_function_number = lambda value, g_idx=g_idx, s_idx=c_idx, slider=False: self.send_slider_value(g_idx,s_idx,value,slider)
                    cb_function_slider = lambda value, g_idx=g_idx, s_idx=c_idx, slider=True: self.send_slider_value(g_idx,s_idx,value,slider)
                    slider.valueChanged.connect(cb_function_slider)
                    spinbox.valueChanged.connect(cb_function_number)
               

                    if 'diff' in g_layout and c_idx == 0:

                        sublayout.addWidget(labelfirst)
                        sublayout.addWidget(slider)
                        sublayout.addWidget(labellast)
                        layout.addWidget(spinbox)
                        layout.addLayout(sublayout)
                    else:
                        layout.addWidget(labelfirst)
                        layout.addWidget(slider)
                        layout.addWidget(labellast)
                        layout.addWidget(spinbox)

                    layout.setAlignment(slider, Qt.AlignHCenter)
                    layout.setAlignment(spinbox, Qt.AlignHCenter)

                    layout_cluster.addLayout(layout)

                    slider_group['slider'] = slider
                    slider_group['number'] = spinbox
                    slider_group['max_label'] = max_label
                    slider_group['min_label'] = min_label
                    control_group['sliders'].append(slider_group)

                    group_layout.addLayout(layout_cluster)

                on_button=QPushButton()
                on_button.setCheckable(True)
                on_button.setText("Off")

                if self.settings['channel_states'][idx]:
                    on_button.toggle()
                    on_button.setText("On")

                on_button.clicked.connect(lambda state, g_idx=g_idx: self.send_channel_state(g_idx,state))

                group_layout.addWidget(on_button)

                row_layout.addLayout(group_layout)
                row_layout.addSpacing(20)

                control_group['on_off'] = on_button
                self.sliders.append(control_group)

                g_idx+=1

            all_rows_layout.addLayout(row_layout)

        sliderbox.addLayout(all_rows_layout)

                    


        # for num_channels_row in self.settings['num_channels']:
        #     row_layout = QHBoxLayout()
        #     for i in range(num_channels_row):
        #         idx = chan_idx*1

        #         slider_group={'slider':None, 'number':None, 'on_off':None}
        #         layout_cluster = QVBoxLayout()

        #         layout = QVBoxLayout()
        #         layout.setAlignment(Qt.AlignHCenter)

        #         slider = QSlider(Qt.Vertical)
        #         slider.setMinimum(self.settings['min_pressure'][idx]*10.0)
        #         slider.setMaximum(self.settings['max_pressure'][idx]*10.0)
        #         slider.setValue(0)
        #         slider.setTickPosition(QSlider.TicksRight)
        #         slider.setTickInterval(20)

        #         spinbox = QDoubleSpinBox()
        #         spinbox.setMinimum(self.settings['min_pressure'][idx])
        #         spinbox.setMaximum(self.settings['max_pressure'][idx])
        #         spinbox.setValue(0)
        #         spinbox.setDecimals(1)
        #         spinbox.setSingleStep(0.1)

        #         cb_function_curr = lambda value, idx=idx, slider=False: self.send_slider_value(idx,value,slider)
        #         cb_function_curr2 = lambda value, idx=idx, slider=True: self.send_slider_value(idx,value,slider)
        #         slider.valueChanged.connect(cb_function_curr2)
        #         spinbox.valueChanged.connect(cb_function_curr)

        #         labelmax = QLabel()
        #         labelmax.setAlignment(Qt.AlignCenter)
        #         labelmax.setText("%0.1f"%(self.settings['max_pressure'][idx]))

        #         labelmin = QLabel()
        #         labelmin.setAlignment(Qt.AlignCenter)
        #         labelmin.setText("%0.1f"%(self.settings['min_pressure'][idx]))
              

        #         layout.addWidget(labelmax)
        #         layout.addWidget(slider)
        #         layout.addWidget(labelmin)
        #         layout.addWidget(spinbox)

        #         layout.setAlignment(slider, Qt.AlignHCenter)
        #         layout.setAlignment(spinbox, Qt.AlignHCenter)

        #         label = QLabel()
        #         label.setText("Chan. %d"%(chan_idx+1))
        #         label.setAlignment(Qt.AlignCenter)
        #         layout_cluster.addWidget(label)
        #         layout_cluster.addLayout(layout)

        #         slider_group['slider'] = slider
        #         slider_group['number'] = spinbox

        #         on_button=QPushButton()
        #         on_button.setCheckable(True)
        #         on_button.setText("Off")

        #         if self.settings['channel_states'][idx]:
        #             on_button.toggle()
        #             on_button.setText("On")

        #         on_button.clicked.connect(lambda state, idx=idx: self.send_channel_state(idx,state))

        #         slider_group['on_off'] = on_button

        #         layout_cluster.addWidget(on_button)

        #         row_layout.addLayout(layout_cluster)
        #         row_layout.addSpacing(20)

        #         self.sliders.append(slider_group)

        #         chan_idx+=1

        #     all_rows_layout.addLayout(row_layout)

        # sliderbox.addLayout(all_rows_layout)

            

            #self._widget.setLayout(sliderbox)

    def get_settings(self):
        settings={}
        settings['channel_states'] = rospy.get_param('/config_node/channels/states',[1,1,1,1,1,1,1,1,1,1])
        settings['num_channels'] = rospy.get_param('/pressure_control/num_channels',[])
        settings['transitions'] = rospy.get_param('/config_node/transitions',1.0)
        settings['gui_config'] = rospy.get_param('/config_node/gui_config',None)
        maxp = rospy.get_param('/config_node/max_pressure',40.0)
        minp = rospy.get_param('/config_node/min_pressure',-40.0)
        settings['num_channels_tot']   = sum(settings['num_channels'])

        if isinstance(maxp, list):
            settings['max_pressure'] = maxp
        elif maxp is not None:
            settings['max_pressure'] = [maxp]*settings['num_channels_tot']


        if isinstance(minp, list):
            settings['min_pressure'] = minp
        elif minp is not None:
            settings['min_pressure'] = [minp]*settings['num_channels_tot']

        settings['pressures'] = [0]*settings['num_channels_tot']

        # If GUI Config is not located, default to rows of pressure sliders for each hardware device
        if settings['gui_config'] is None:
            chan_idx = 0
            gui_config=[]
            for num_channels in settings['num_channels']:
                row=[]
                for idx in range(num_channels):
                    row.append({'channels':[chan_idx], 'layout':'single'})
                    chan_idx+=1
                gui_config.append(row)
            
            settings['gui_config'] = gui_config
            print(gui_config)
        
        settings['gui_config_flat'] = []
        for row in settings['gui_config']:
            for group in row:
                settings['gui_config_flat'].append(group)

        return settings

    def set_transition_value(self, value):
        self.settings['transitions'] = value
        rospy.set_param('/config_node/transitions',self.settings['transitions'])

    def set_pressure_zero(self, value):
        self.settings['pressures'] = [0]*self.settings['num_channels_tot']
        self.send_command(command='set', args=[self.settings['transitions']]+self.settings['pressures'])

        for slider_group in self.sliders:
            for slider in slider_group['sliders']:
                #slider_group['slider'].setValue(0)
                slider['number'].setValue(0)
        

    def send_slider_value(self, g_idx, s_idx, value, slider):

        if slider:
            press = self.sliders[g_idx]['sliders'][s_idx]['slider'].value()/10.0
            self.sliders[g_idx]['sliders'][s_idx]['number'].setValue(press)
        else:
            press = self.sliders[g_idx]['sliders'][s_idx]['number'].value()
            self.sliders[g_idx]['sliders'][s_idx]['slider'].setValue(press*10.0)

        print(press)


        config_set = self.settings['gui_config_flat'][g_idx]

        if 'diff' in config_set['layout']:
            diff_p = self.sliders[g_idx]['sliders'][0]['number'].value()
            main_p = self.sliders[g_idx]['sliders'][1]['number'].value()

            if 'inv' in config_set['layout']:
                p1 = main_p+diff_p
                p2 = main_p-diff_p
            
            else:
                p1 = main_p-diff_p
                p2 = main_p+diff_p

            self.settings['pressures'][config_set['channels'][0]] = p1
            self.settings['pressures'][config_set['channels'][1]] = p2

            maxp1=self.settings['max_pressure'][config_set['channels'][0]]
            maxp2=self.settings['max_pressure'][config_set['channels'][1]]

            avg_maxp = sum([maxp1, maxp2])/2.0
            max_bound = avg_maxp - main_p
            


            self.update_slider_bounds(g_idx,0,[-max_bound,max_bound])

            print(g_idx, s_idx, main_p, diff_p)
        
        else:
            self.settings['pressures'][config_set['channels'][s_idx]]=press

        self.send_command(command='set', args=[self.settings['transitions']]+self.settings['pressures'])


    def send_channel_state(self, g_idx, state):

        if state:
            self.sliders[g_idx]['on_off'].setText("On")
            state_num=1
        else:
            self.sliders[g_idx]['on_off'].setText("Off")
            state_num=0

        config_set = self.settings['gui_config_flat'][g_idx]
        channels = config_set['channels']

        for channel in channels:
            self.settings['channel_states'][channel] = state_num

        self.send_command(command='chan', args=self.settings['channel_states'])

        rospy.set_param('/config_node/channels/states',self.settings['channel_states'])
         

    def set_graph_state(self, value):

        if value:
            on_off_str='on'
            self.graph_button.setText("Graph ON")
        else:
            on_off_str='off'
            self.graph_button.setText("Graph OFF")

        self.send_command(command=on_off_str, args=[])


    def set_reset(self, value):
        self.send_command(command='mode', args=[3])


    def send_command(self, command, args, wait_for_ack=False):
        if self._client_connected:
            # Send commands to the command server and wait for things to be taken care of
            goal = pressure_controller_ros.msg.CommandGoal(command=command, args=args, wait_for_ack = False)
            self._client.send_goal(goal)
            self._client.wait_for_result()

            if not self._client.get_result():
                raise ('Something went wrong and a setting was not validated')
                pass
            else:
                pass
        else:
            print(command, args)


    def update_slider_bounds(self, g_idx, s_idx, values):
        self.sliders[g_idx]['sliders'][s_idx]['slider'].setMinimum(values[0]*10.0)
        self.sliders[g_idx]['sliders'][s_idx]['number'].setMinimum(values[0])

        self.sliders[g_idx]['sliders'][s_idx]['slider'].setMaximum(values[1]*10.0)
        self.sliders[g_idx]['sliders'][s_idx]['number'].setMaximum(values[1])

        self.sliders[g_idx]['sliders'][s_idx]['min_label'].setText("%0.1f"%(values[0]))
        self.sliders[g_idx]['sliders'][s_idx]['max_label'].setText("%0.1f"%(values[1]))



    def shutdown_sliders(self):
        print("")
        print("Final Pressures:")
        print(self.settings['pressures'])
        print("")

        self.set_graph_state(False)

        for slider_group in self.sliders:
            for slider in slider_group['sliders']:
                slider['slider'].valueChanged.disconnect()
                slider['number'].valueChanged.disconnect()

            on_off = slider_group['on_off']
            on_off.clicked.disconnect()
        
    def save_mcu_settings(self):
        self.send_command(command='save', args=[])


    def shutdown_plugin(self):
        self._client.cancel_all_goals()
        self.shutdown_sliders()
        self.save_mcu_settings()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
    #    pass