from __future__ import print_function
from genericpath import exists
import time
import rospy
import rospkg
import actionlib
import importlib
import csv
import sys
import os
import copy
import numpy as np

from geometry_msgs.msg import Wrench, WrenchStamped
from tf2_msgs.msg import TFMessage
#from controller_manager.msg import ControllerState
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix, quaternion_matrix

from simple_ur_move.controller_handler import ControllerHandler
from simple_ur_move.cartesian_trajectory_handler import CartesianTrajectoryHandler
from simple_ur_move.twist_handler import TwistHandler

try:
    filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')
except:
    filepath_config = "../../config"

class RobotController:
    '''
    Hardware interface that can control a robot via ROS controllers

    Parameters
    ----------
    robot_name : str
        Name of the robot. This name must match the prefix of the robot's
        controller topics
    debug : bool
        Turn on debugging print statements
    '''
    def __init__(self, robot_name="", debug=False):
        self.robot_name = robot_name
        self.debug=debug
         # Subscribe to the wrench and tf topics
        rospy.Subscriber('/wrench', WrenchStamped, self.update_wrench)
        rospy.Subscriber('/tf', TFMessage, self.update_tool_pose)

        self.wrench_pub = rospy.Publisher('/wrench_balanced', WrenchStamped, queue_size=10)
        self.tf_pub = rospy.Publisher('/tf_balanced', TFMessage, queue_size=10)

        self.controller_handler = ControllerHandler(self.robot_name)
        self.current_controllers = []

        self.force_curr = None
        self.torque_curr = None
        self.set_offsets(None)
    

    def load_controller(self, controller):
        '''
        Load a ROS controller

        Parameters
        ----------
        controller : str
            Name of the controller to load

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        resp=self.controller_handler.load_controller(controller)            
        return resp


    def unload_controller(self, controller):
        '''
        Unload a ROS controller

        Parameters
        ----------
        controller : str
            Name of the controller to unload

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        return self.controller_handler.unload_controller(controller)
        

    def switch_controller(self, start_controllers, stop_controllers, strictness=1, start_asap=False, timeout=0):
        '''
        Switch ROS controllers

        Parameters
        ----------
        start_controllers : list
            Names of the controllers to start
        stop_controllers : list
            Names of the controllers to stop
        strictness : int
            Strictness of controller switching
        start_asap : bool
            Decide whether controllers should be started immediately
        timeout : int
            Timeout (in seconds)

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        resp = self.controller_handler.switch_controller(start_controllers,
                                                         stop_controllers,
                                                         strictness,
                                                         start_asap,
                                                         timeout)
        if resp is not None:
            self.current_controllers = self.controller_handler.get_controllers_with_state('running')
        return resp
        

    def set_controller(self,controller):
        '''
        Set which ROS controller is started, and stop all others

        Parameters
        ----------
        controller : str
            Name of the controller to start

        Returns
        -------
        response : str
            Service response from the controller manager
        '''
        resp = self.controller_handler.set_controller(controller)

        if resp is not None:
            self.current_controllers = self.controller_handler.get_controllers_with_state('running')

        return resp


    def set_offsets(self, offsets):
        '''
        Set the internal F/T and Pose offsets

        Parameters
        ----------
        offsets : dict
            Offset dict with "force", "torque", "position",
            and "orientation" keys
        '''
        if offsets is None:
            self.offsets = {'force':[0,0,0],
                        'torque':[0,0,0],
                        'position':[0,0,0],
                        'orientation':[0,0,0]}
        else:
            self.offsets = offsets


    def balance(self,type):
        if type=='pose':
            self.balance_pose()
        elif type=='ft':
            self.balance_ft()
        else:
            rospy.logerr("Incorrect balance type")


    def balance_pose(self):
        '''
        Zero the internal pose offsets
        '''
        offset = self.get_offsets()
        offset['position'] = self.position_raw
        offset['orientation'] = self.orientation_raw
        self.set_offsets(offset)


    def balance_ft(self):
        '''
        Zero the internal F/T offsets
        '''
        offset = self.get_offsets()
        offset['force'] = self.force_raw
        offset['torque'] = self.torque_raw
        self.set_offsets(offset)


    def get_offsets(self):
        '''
        Get the internal F/T and Pose offsets

        Returns
        ----------
        offsets : dict
            Offset dict with "force", "torque", "position",
            and "orientation" keys
        '''
        return self.offsets


    def update_wrench(self,data):
        '''
        Update the internal value of the wrench. The wrench is converted
        from the tool frame to the world frame, then balanced via offsets.
        The balanced wrench is published in the ``/wrench_balanced`` topic. 

        Parameters
        ----------
        data : geometry_msgs/Wrench
            Wrench message
        '''
        if not hasattr(self, 'orientation_raw_quat'):
            return
        
        # Convert the wrench data to the world frame
        rotation = np.array(quaternion_matrix(self.orientation_raw_quat))
        if self.debug:
            print(self.orientation_raw)
            print(rotation[0:3,0:3])

        wrench = data.wrench
        force_direct = [wrench.force.x, wrench.force.y, wrench.force.z]
        torque_direct = [wrench.torque.x, wrench.torque.y, wrench.torque.z]

        self.force_raw  = np.matmul(rotation[0:3,0:3],force_direct).tolist()
        self.torque_raw = np.matmul(rotation[0:3,0:3],torque_direct).tolist()

        # Calculate the current wrench (balanced) based on offsets
        self.force_curr = [x-off for x,off in zip(self.force_raw,self.offsets['force'])]
        self.torque_curr = [x-off for x,off in zip(self.torque_raw,self.offsets['torque'])]

        # Send a message to the output topic
        data_out = copy.deepcopy(data)
        data_out.wrench.force.x = self.force_curr[0]
        data_out.wrench.force.y = self.force_curr[1]
        data_out.wrench.force.z = self.force_curr[2]
        data_out.wrench.torque.x = self.torque_curr[0]
        data_out.wrench.torque.y = self.torque_curr[1]
        data_out.wrench.torque.z = self.torque_curr[2]

        self.wrench_pub.publish(data_out)



    def update_tool_pose(self,data):
        '''
        Update the internal value of the tool pose. Saves a copy of
        the cartesian position (in m) and euler angle orientation (in rad).
        The pose is then balanced via offsets. The balanced wrench is
        published in the ``/tf_balanced`` topic. 

        Parameters
        ----------
        data : tf2_msgs/TFMessage
            Wrench message
        '''
        tf = data.transforms[0]

        if "tool0_controller" not in tf.child_frame_id:
            return

        # Unpack the tool pose
        transform = tf.transform
        self.position_raw = [transform.translation.x, transform.translation.y, transform.translation.z]
        self.orientation_raw = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        self.orientation_raw_quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]

        # Calculate the current pose (balanced) based on offsets
        self.position_curr = [x-off for x,off in zip(self.position_raw,self.offsets['position'])]
        self.orientation_curr = [x-off for x,off in zip(self.orientation_raw,self.offsets['orientation'])]

        # Send a message to the output topic
        data_out = copy.deepcopy(data)
        data_out.transforms[0].transform.translation.x = self.position_curr[0]
        data_out.transforms[0].transform.translation.y = self.position_curr[1]
        data_out.transforms[0].transform.translation.z = self.position_curr[2]
        data_out.transforms[0].transform.rotation.x = self.orientation_curr[0]
        data_out.transforms[0].transform.rotation.y = self.orientation_curr[1]
        data_out.transforms[0].transform.rotation.z = self.orientation_curr[2]
        data_out.transforms[0].transform.rotation.w = 0

        self.tf_pub.publish(data_out)


    def get_twist(self, linear, angular):
        '''
        Build a twist message from vectors

        Parameters
        ----------
        linear : list
            The linear twist components [x,y,z]
        angular : list
            The angular twist components [x,y,z]

        Returns
        -------
        twist : geometry_msgs/Twist
            The resulting twist message
        '''
        linear_out = Vector3()
        linear_out.x=linear[0]
        linear_out.y=linear[1]
        linear_out.z=linear[2]

        angular_out = Vector3()
        angular_out.x=angular[0]
        angular_out.y=angular[1]
        angular_out.z=angular[2]

        twist_out = Twist()
        twist_out.linear=linear_out
        twist_out.angular=angular_out

        return twist_out


    def set_jog(self, linear, angular):
        '''
        Set the Jog speed

        Parameters
        ----------
        linear : list
            The linear twist components [x,y,z]
        angular : list
            The angular twist components [x,y,z]
        '''
        handler=TwistHandler(self.robot_name, debug=True)
        handler.load_config('jog_control.yaml', filepath_config)

        self.set_speed_slider(1.0)
        handler.set_twist({'linear':linear,'angular':angular})


    def set_pose(self, pose, time=5.0):
        '''
        Set the pose of the robot

        Parameters
        ----------
        pose : dict
            The pose dictionary with position and orientation components
        time : float
            Time to take (in seconds)
        '''
        traj_handler = CartesianTrajectoryHandler(self.robot_name,
                "pose_based_cartesian_traj_controller", self.debug)
        traj_handler.load_config('pose_control.yaml', filepath_config)
        
        traj_handler.set_initialize_time(time)

        point = {}
        point['position']=pose['position']
        point['orientation']=pose['orientation']

        self.set_speed_slider(1.0)
        traj_handler.go_to_point(point)


    def set_speed_slider(self, fraction):
        '''
        Set the speed slider fraction

        Parameters
        ----------
        fraction : float
            Slider fraction to set (0.02 to 1.00)

        Returns
        -------
        result : srv
            The result of the service call
        '''
        return self.controller_handler.set_speed_slider(fraction)


    def play_program(self):
        '''
        Start the program on the teach pendant.
        This ony works if you are in remote control mode

        Returns
        -------
        result : srv
            The result of the service call
        '''
        return self.controller_handler.play_program()


    def stop_program(self):
        '''
        Stop the program on the teach pendant.
        This ony works if you are in remote control mode

        Returns
        -------
        result : srv
            The result of the service call
        '''
        return self.controller_handler.stop_program()


    def shutdown(self):
        '''
        Shutdown gracefully
        '''
        if "twist_controller" in self.current_controllers:
            self.set_jog([0,0,0],[0,0,0])



class DataLogger:
    '''
    Log data to csv files.

    Parameters
    ----------
    filename : str
        Filename to save to. *Note: files will be saved with suffixes
        corresponding to the name of the data being saved*

    config : dict
        Configuration, including the topic map
    
    Raises
    ------
    ValueError
        If the topic_map is invalid
    '''
    def __init__(self, filename, config, overwrite=False):
        
        self.config = config
        
        self.topic_map=config.get('topic_map', None)
        self._validate_topic_map(self.topic_map)

        if overwrite:
            self.filename=filename
        else:
            self.filename = self.get_unique_filename(filename)

        print('Saving Data: %s'%(self.filename))

        self.running = False
        self.logging = False
        self.curr_time = 0
        self.names = []


    def get_unique_filename(self, filename):
        if len(self.names)<1:
            return filename

        file_stripped, file_stripped_ext = os.path.splitext(filename)
        filename_duplicate = True
        i=1
        while filename_duplicate:
            filename_duplicate = False
            for name in self.names:
                curr_filename = "%s_%03d_%s%s"%(file_stripped, i, self.names[0], file_stripped_ext)
                if os.path.exists(curr_filename):
                    filename_duplicate = True
                    i+=1
                    break
            

        return "%s_%03d%s"%(file_stripped, i, file_stripped_ext)
        
        



    def _validate_topic_map(self, topic_map):
        '''
        Check that the topic map was defined and has no duplicates.

        Parameters
        ----------
        topic_map : list of dicts
            The topic map to validate
        
        Raises
        ------
        ValueError
            If the map is None or contains duplicate names
        '''
        if topic_map is None:
            raise ValueError("Topic map was not defined in the config file.")

        names = []
        for map_val in topic_map:
            names.append(map_val['name'])

        if len(set(names)) != len(names):
            raise ValueError("Duplicate log names were found in the config")

        self.names = names
        

    def _log_data(self,data, map_val):
        '''
        Log data from a message based on specification in the topic map.

        Parameters
        ----------
        data : Any()
            Message from a ROS topic
        map_val : dict
            The value of the topic map to use
        '''
        if self.logging:
            try:
                time = eval('data.'+map_val['header_field']).stamp.to_sec()
            except AttributeError:
                rospy.loginfo("Incorrect header definition in config")
                raise
            data_expanded = self._expand_data(data, map_val)
            data_expanded.insert(0,time-self.curr_time)
            key = map_val['name']
            self.loggers[key].writerow(data_expanded)


    def _expand_data(self, data, map_val):
        '''
        Expand data from a message based on specification in the topic map.

        Parameters
        ----------
        data : Any()
            Message from a ROS topic
        map_val : dict
            The value of the topic map to use
        
        Returns
        -------
        data_expanded : list
            Data retrieved from the topic
        '''
        data_gotten=eval('data.'+map_val['fields'])
            
        subfields = map_val.get('subfields', None)
        if subfields is not None:
            data_out = []
            for subfield in subfields:
                curr_val = eval('data_gotten.'+subfield)
                data_out.append(curr_val)

            return data_out
        else:
            return [data_gotten]


    def _write_header(self, map_val):
        '''
        Write the header labels to the file

        Parameters
        ----------
        map_val : dict
            The value of the topic map to use
        '''
        name = map_val['name']
        units = map_val['units']
        if units is None or not units:
            units = ""

        header=[]
        header.append('Time (s)')

        subfields = map_val.get('subfields',None)
        if subfields is not None:
            for subfield in subfields:
                if units:
                    header.append("%s_%s (%s)"%(name, subfield, units))
                else:
                    header.append("%s_%s"%(name, subfield))
        else:
            if units:
                header.append("%s (%s)"%(name, units))
            else:
                header.append(name)

        self.loggers[name].writerow(header)


    def start(self):
        '''
        Start logging data.
        '''
        if not self.running:
            dirname = os.path.dirname(self.filename)
            if not os.path.exists(dirname):
                os.makedirs(dirname)

            self.subscribers={}
            self.loggers={}
            self.logfiles={}
            for map_val in self.topic_map:
                key = map_val['name']
                topic = map_val['topic']
                message_type = map_val['message_type']
                msg_type = message_type.rsplit('.',1)
                msg_class = getattr(importlib.import_module(msg_type[0]), msg_type[1])
                
                file_stripped = os.path.splitext(self.filename)[0]
                file_stripped_ext = os.path.splitext(self.filename)[1]
                curr_filename = "%s_%s%s"%(file_stripped, key, file_stripped_ext)

                self.subscribers[key] = rospy.Subscriber('/'+topic,
                    msg_class,
                    lambda data, map_val = map_val: self._log_data(data, map_val))
                #self.subscribers[key] = GenericMessageSubscriber('/'+topic, lambda data, map_val = map_val: self._log_data(data, map_val))
                self.logfiles[key] = open(curr_filename, 'w')
                self.loggers[key]  = csv.writer(self.logfiles[key])

                self._write_header(map_val)
            
            self.curr_time = rospy.get_rostime().to_sec()
            self.running = True
            self.logging = True
            
        else:
            rospy.loginfo("Loggers were already running")

        
    def pause(self):
        '''
        Pause logging.
        '''
        if self.running:
            self.logging = False
        else:
            rospy.loginfo("No loggers were running")


    def resume(self):
        '''
        Resume logging.
        '''
        if self.running:
            self.logging = True
        else:
            rospy.loginfo("No loggers were running")


    def stop(self):
        '''
        Stop logging data.
        '''
        if self.running:
            # Stop Logging
            self.logging = False

            # Cancel all subscribers
            for key in self.subscribers:
                self.subscribers[key].unregister()
                self.subscribers[key]

            # Close all logfiles
            for key in self.logfiles:
                self.logfiles[key].close()
            
            # Reset variables
            del self.subscribers
            del self.logfiles
            del self.loggers

            self.subscribers = {}
            self.logfiles = {}
            self.loggers = {}

            self.running = False
        else:
            rospy.loginfo("No loggers were running")


    def shutdown(self):
        '''
        Shutdown gracefully.
        '''
        rospy.loginfo("Shutting Down")
        self.stop()



class GenericMessageSubscriber(object):
    '''
    ROS subscriber that automatically retrieves the message
    type of the topic you want to subscribe to.

    Parameters
    ----------
    topic_name : str
        The topic to subscribe to

    callback : function
        Callback function to use
    '''
    def __init__(self, topic_name, callback):
        self._binary_sub = rospy.Subscriber(
            topic_name, rospy.AnyMsg, self.generic_message_callback)
        self._callback = callback


    def generic_message_callback(self, data):
        '''
        Callback function that handles any message type and passes that 
        data to the subscriber's callback function.

        Parameters
        ----------
        data : Any()
            Data from a ROS message
        '''
        assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
        connection_header =  data._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        msg_class = getattr(importlib.import_module(ros_pkg), msg_type)
        msg = msg_class().deserialize(data._buff)
        self._callback(msg)
    

    def unregister(self):
        '''
        Unregister the subscriber from the topic.
        '''
        self._binary_sub.unregister()