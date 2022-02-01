import os
import rospy
import rospkg
import actionlib
import copy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import qDebug, Qt
from python_qt_binding.QtWidgets import QWidget, QLayout, QVBoxLayout, QHBoxLayout, QSlider, QDoubleSpinBox, QPushButton, QLabel
from qt_gui_py_common.simple_settings_dialog import SimpleSettingsDialog

import pressure_controller_ros.msg

#try:
#    from pyqtgraph_data_plot import PyQtGraphDataPlot
#except ImportError:
#    qDebug('[DEBUG] rqt_plot.plot: import of PyQtGraphDataPlot failed (trying other backends)')
#    PyQtGraphDataPlot = None

#try:
#    from mat_data_plot import MatDataPlot
#except ImportError:
#    qDebug('[DEBUG] rqt_plot.plot: import of MatDataPlot failed (trying other backends)')
#    MatDataPlot = None

#try:
#    from qwt_data_plot import QwtDataPlot
#except ImportError:
#    qDebug('[DEBUG] rqt_plot.plot: import of QwtDataPlot failed (trying other backends)')
#    QwtDataPlot = None

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
        ui_file = os.path.join(rospkg.RosPack().get_path('virtual_instron_gui'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')

        self.settings=self.get_settings()
        
        self.init_sliders()
        #self.init_config()

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


    # def init_config(self):
    #     self._plot_type_index = 0
    #     self.plot_types = [
    #         {
    #             'title': 'PyQtGraph',
    #             'widget_class': PyQtGraphDataPlot,
    #             'description': 'Based on PyQtGraph\n- installer: http://luke.campagnola.me/code/pyqtgraph',
    #             'enabled': True,#PyQtGraphDataPlot is not None,
    #         },
    #         {
    #             'title': 'MatPlot',
    #             'widget_class': MatDataPlot,
    #             'description': 'Based on MatPlotLib\n- needs most CPU\n- needs matplotlib >= 1.1.0\n- if using PySide: PySide > 1.1.0',
    #             'enabled': True,#MatDataPlot is not None,
    #         },
    #         {
    #             'title': 'QwtPlot',
    #             'widget_class': QwtDataPlot,
    #             'description': 'Based on QwtPlot\n- does not use timestamps\n- uses least CPU\n- needs Python Qwt bindings',
    #             'enabled': True,#QwtDataPlot is not None,
    #         },
    #     ]

    def init_sliders(self):
        sliderbox = self._widget.findChild(QLayout,'Sliders')

        graph_button=QPushButton()
        graph_button.setCheckable(True)
        graph_button.setText("Graph Off")
        graph_button.toggle()
        graph_button.clicked.connect(self.set_graph_state)
        self.graph_button = graph_button

        firstCol = QVBoxLayout()
        firstCol.addWidget(graph_button)

        sliderbox.addLayout(firstCol)

        self.sliders = []


        all_rows_layout = QVBoxLayout()
        chan_idx = 0
        for num_channels_row in self.settings['num_channels']:
            row_layout = QHBoxLayout()
            for i in range(num_channels_row):
                idx = chan_idx*1

                slider_group={'slider_p':None, 'number_p':None,
                              'slider_i':None, 'number_i':None,
                              'slider_d':None, 'number_d':None,
                              'on_off':None}

                layout_cluster = QVBoxLayout()
                slider_cluster = QHBoxLayout()
                label = QLabel()
                label.setText("Chan. %d"%(idx+1))
                label.setAlignment(Qt.AlignCenter)
                layout_cluster.addWidget(label)
                for j in range(3):
                    layout = QVBoxLayout()
                    layout.setAlignment(Qt.AlignHCenter)

                    if j==0:
                        maxrange=1.0
                    elif j==1:
                        maxrange=10
                    elif j==2:
                        maxrange=10

                    slider = QSlider(Qt.Vertical)
                    slider.setMinimum(0)
                    slider.setMaximum(maxrange*100)
                    slider.setValue(self.settings['pid_gains'][chan_idx][j]*100)
                    slider.setTickPosition(QSlider.TicksRight)
                    slider.setTickInterval(maxrange/100.0)

                    spinbox = QDoubleSpinBox()
                    spinbox.setMinimum(0)
                    spinbox.setMaximum(maxrange)
                    spinbox.setValue(self.settings['pid_gains'][chan_idx][j])
                    spinbox.setDecimals(2)
                    spinbox.setSingleStep(maxrange/100.0)

                    cb_function_number = lambda value, idx=idx, gain_idx=j, slider=False: self.send_slider_value(idx, gain_idx, value, slider)
                    cb_function_slider = lambda value, idx=idx, gain_idx=j, slider=True:  self.send_slider_value(idx, gain_idx, value, slider)

                    slider.valueChanged.connect(cb_function_slider)
                    spinbox.valueChanged.connect(cb_function_number)


                    label = QLabel()
                    label.setAlignment(Qt.AlignCenter)

                    if j==0:
                        slider_group['slider_p'] = slider
                        slider_group['number_p'] = spinbox
                        label.setText("P")
                    elif j==1:
                        slider_group['slider_i'] = slider
                        slider_group['number_i'] = spinbox
                        label.setText("I")
                    elif j==2:
                        slider_group['slider_d'] = slider
                        slider_group['number_d'] = spinbox
                        label.setText("D")

                    labelmax = QLabel()
                    labelmax.setAlignment(Qt.AlignCenter)
                    labelmax.setText("%0.1f"%(maxrange))

                    labelmin = QLabel()
                    labelmin.setAlignment(Qt.AlignCenter)
                    labelmin.setText("0")

                    
                    layout.addWidget(label)
                    layout.addWidget(labelmax)
                    layout.addWidget(slider, Qt.AlignHCenter)
                    layout.addWidget(labelmin)
                    layout.addWidget(spinbox, Qt.AlignHCenter)
                    layout.setAlignment(slider, Qt.AlignHCenter)
                    layout.setAlignment(spinbox, Qt.AlignHCenter)

                    slider_cluster.addLayout(layout)

                

                on_button=QPushButton()
                on_button.setCheckable(True)
                on_button.setText("Off")

                if self.settings['channel_states'][chan_idx]:
                    on_button.toggle()
                    on_button.setText("On")

                on_button.clicked.connect(lambda state, idx=idx: self.send_channel_state(idx,state))

                slider_group['on_off'] = on_button

                layout_cluster.addLayout(slider_cluster)
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
        settings['num_channels_tot']   = sum(settings['num_channels'])
        settings['pid_gains']  = rospy.get_param('/config_node/PID/values',[[0,0,0]]*settings['num_channels_tot'])

        if isinstance(settings['pid_gains'][0], list):
            settings['pid_gains'] = settings['pid_gains']
        else:
            settings['pid_gains'] = [settings['pid_gains']]*settings['num_channels_tot']

        print(settings['pid_gains'])

        return settings

    def send_slider_value(self, idx, gain_idx, value, slider):

        if gain_idx ==0:
            slider_str = 'slider_p'
            number_str = 'number_p'
        elif gain_idx ==1:
            slider_str = 'slider_i'
            number_str = 'number_i'
        elif gain_idx ==2:
            slider_str = 'slider_d'
            number_str = 'number_d'

        if slider:
            cur_val = self.sliders[idx][slider_str].value()/100.0
            self.sliders[idx][number_str].setValue(cur_val)
        else:
            cur_val = self.sliders[idx][number_str].value()
            self.sliders[idx][slider_str].setValue(cur_val*100.0)
        
        self.settings['pid_gains'][idx][gain_idx] = cur_val

        self.send_command(command='pid', args=[idx]+self.settings['pid_gains'][idx])


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
        rospy.set_param('/config_node/PID/values',self.settings['pid_gains'])
        print("")
        print("Final PID Gains:")
        print(self.settings['pid_gains'])
        print("")
        print("These settings were saved on the pressure controller")
        print("")
        print("To ensure these parameters are always recalled, for now you need to:")
        print("    - Copy these settings into 'pid/values' in all your control config files")

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
    #    dialog = SimpleSettingsDialog(title='Plot Options')
    #    dialog.add_exclusive_option_group(title='Plot Type', options=self.plot_types, selected_index=self._plot_type_index)
    #    plot_type = dialog.get_settings()[0]
    #    if plot_type is not None:
    #        qDebug(str(plot_type['selected_index']))
    #    if plot_type is not None and plot_type['selected_index'] is not None and self._plot_type_index != plot_type['selected_index']:
    #        qDebug('[DEBUG] SWITCHING')
    #        self._plot_type_index=plot_type['selected_index']