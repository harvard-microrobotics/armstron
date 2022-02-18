import os
import rospy
import rospkg
import actionlib
import copy

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

        reset_button=QPushButton()
        reset_button.setCheckable(False)
        reset_button.setText("Reset")
        reset_button.clicked.connect(self.set_reset)

        self.graph_button = graph_button
        self.reset_button = reset_button

        firstCol.addWidget(graph_button)
        firstCol.addWidget(reset_button)

        firstCol.setAlignment(graph_button,Qt.AlignVCenter)
        firstCol.setAlignment(reset_button,Qt.AlignVCenter)

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

        chan_idx = 0
        for num_channels_row in self.settings['num_channels']:
            row_layout = QHBoxLayout()
            for i in range(num_channels_row):
                idx = chan_idx*1

                slider_group={'slider':None, 'number':None, 'on_off':None}
                layout_cluster = QVBoxLayout()

                layout = QVBoxLayout()
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

                cb_function_curr = lambda value, idx=idx, slider=False: self.send_slider_value(idx,value,slider)
                cb_function_curr2 = lambda value, idx=idx, slider=True: self.send_slider_value(idx,value,slider)
                slider.valueChanged.connect(cb_function_curr2)
                spinbox.valueChanged.connect(cb_function_curr)

                labelmax = QLabel()
                labelmax.setAlignment(Qt.AlignCenter)
                labelmax.setText("%0.1f"%(self.settings['max_pressure'][idx]))

                labelmin = QLabel()
                labelmin.setAlignment(Qt.AlignCenter)
                labelmin.setText("%0.1f"%(self.settings['min_pressure'][idx]))
              

                layout.addWidget(labelmax)
                layout.addWidget(slider)
                layout.addWidget(labelmin)
                layout.addWidget(spinbox)

                layout.setAlignment(slider, Qt.AlignHCenter)
                layout.setAlignment(spinbox, Qt.AlignHCenter)

                label = QLabel()
                label.setText("Chan. %d"%(chan_idx+1))
                label.setAlignment(Qt.AlignCenter)
                layout_cluster.addWidget(label)
                layout_cluster.addLayout(layout)

                slider_group['slider'] = slider
                slider_group['number'] = spinbox

                on_button=QPushButton()
                on_button.setCheckable(True)
                on_button.setText("Off")

                if self.settings['channel_states'][idx]:
                    on_button.toggle()
                    on_button.setText("On")

                on_button.clicked.connect(lambda state, idx=idx: self.send_channel_state(idx,state))

                slider_group['on_off'] = on_button

                layout_cluster.addWidget(on_button)

                row_layout.addLayout(layout_cluster)
                row_layout.addSpacing(20)

                self.sliders.append(slider_group)

                chan_idx+=1

            all_rows_layout.addLayout(row_layout)

        sliderbox.addLayout(all_rows_layout)

            

            #self._widget.setLayout(sliderbox)

    def get_settings(self):
        settings={}
        settings['channel_states'] = rospy.get_param('/config_node/channels/states',[1,1,1,1,1,1,1,1,1,1])
        settings['num_channels'] = rospy.get_param('/pressure_control/num_channels',[])
        settings['transitions'] = rospy.get_param('/config_node/transitions',1.0)
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

        return settings

    def set_transition_value(self, value):
        self.settings['transitions'] = value
        rospy.set_param('/config_node/transitions',self.settings['transitions'])

    def set_pressure_zero(self, value):
        self.settings['pressures'] = [0]*self.settings['num_channels_tot']
        self.send_command(command='set', args=[self.settings['transitions']]+self.settings['pressures'])

        for slider_group in self.sliders:
            #slider_group['slider'].setValue(0)
            slider_group['number'].setValue(0)
        

    def send_slider_value(self, idx, value, slider):

        if slider:
            press = self.sliders[idx]['slider'].value()/10.0
            self.sliders[idx]['number'].setValue(press)
        else:
            press = self.sliders[idx]['number'].value()
            self.sliders[idx]['slider'].setValue(press*10.0)

        

        self.settings['pressures'][idx]=press

        self.send_command(command='set', args=[self.settings['transitions']]+self.settings['pressures'])


    def send_channel_state(self, idx, state):
        if state:
            self.settings['channel_states'][idx] = 1
            self.sliders[idx]['on_off'].setText("On")
        else:
            self.settings['channel_states'][idx] = 0
            self.sliders[idx]['on_off'].setText("Off")

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



    def shutdown_sliders(self):
        print("")
        print("Final Pressures:")
        print(self.settings['pressures'])
        print("")

        self.set_graph_state(False)

        for slider_group in self.sliders:
            for widget in slider_group:
                if 'on_off' in widget:
                    slider_group[widget].clicked.disconnect()
                else:
                    slider_group[widget].valueChanged.disconnect()
        
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