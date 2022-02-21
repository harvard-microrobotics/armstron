from __future__ import print_function
from genericpath import exists
import time
import rospy
import actionlib
import importlib
import csv
import sys
import os
import copy

from geometry_msgs.msg import Wrench, WrenchStamped
from tf2_msgs.msg import TFMessage
from controller_manager_msgs.srv import LoadController, UnloadController, SwitchController
#from controller_manager.msg import ControllerState
from geometry_msgs.msg import Twist, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion

controller_list = ['scaled_pos_joint_traj_controller', 'pose_based_cartesian_traj_controller', 'twist_controller']

class RobotController:
    '''
    Hardware interface that can control a robot via ROS controllers

    Parameters
    ----------
    robot_name : str
        Name of the robot. This name must match the prefix of the robot's
        controller topics
    '''
    def __init__(self, robot_name=""):
         # Subscribe to the wrench and tf topics
        rospy.Subscriber('/wrench', WrenchStamped, self.update_wrench)
        rospy.Subscriber('/tf', TFMessage, self.update_tool_pose)

        self.wrench_pub = rospy.Publisher('/wrench_balanced', WrenchStamped, queue_size=10)
        self.tf_pub = rospy.Publisher('/tf_balanced', TFMessage, queue_size=10)
        self.jog_pub = rospy.Publisher('/twist_controller/command', Twist, queue_size=10)

        self.controller_list = controller_list

        self.load_controller_name = robot_name+'/controller_manager/load_controller'
        self.unload_controller_name = robot_name+'/controller_manager/unload_controller'
        self.switch_controller_name = robot_name+'/controller_manager/switch_controller'

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
        rospy.wait_for_service(self.load_controller_name)
        try:
            fun = rospy.ServiceProxy(self.load_controller_name, LoadController)
            resp1 = fun(controller)
            return resp1.ok
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


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
        rospy.wait_for_service(self.unload_controller_name)
        try:
            fun = rospy.ServiceProxy(self.unload_controller_name, UnloadController)
            resp1 = fun(controller)
            return resp1.ok
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        

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
        rospy.wait_for_service(self.switch_controller_name)
        try:
            fun = rospy.ServiceProxy(self.switch_controller_name, SwitchController)
            resp1 = fun(start_controllers=start_controllers,
                             stop_controllers=stop_controllers,
                             strictness=strictness,
                             start_asap=start_asap,
                             timeout=timeout)
            return resp1.ok
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        

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

        controllers_to_unload = []

        for ctrl in self.controller_list:
            if controller != ctrl:
                controllers_to_unload.append(ctrl)
                
        self.load_controller(controller)
        self.switch_controller([controller],controllers_to_unload)


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
        Update the internal value of the wrench.

        Parameters
        ----------
        data : geometry_msgs/Wrench
            Wrench message
        '''
        wrench = data.wrench
        self.force_raw = [wrench.force.x, wrench.force.y, wrench.force.z]
        self.torque_raw = [wrench.torque.x, wrench.torque.y, wrench.torque.z]
        self.force_curr = [x-off for x,off in zip(self.force_raw,self.offsets['force'])]
        self.torque_curr = [x-off for x,off in zip(self.torque_raw,self.offsets['torque'])]

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

        Parameters
        ----------
        data : tf2_msgs/TFMessage
            Wrench message
        '''
        tf = data.transforms[0]

        if "tool0_controller" not in tf.child_frame_id:
            return

        transform = tf.transform
        self.position_raw = [transform.translation.x, transform.translation.y, transform.translation.z]
        self.orientation_raw = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
        self.position_curr = [x-off for x,off in zip(self.position_raw,self.offsets['position'])]
        self.orientation_curr = [x-off for x,off in zip(self.orientation_raw,self.offsets['orientation'])]

        #ori_quat = quaternion_from_euler(self.orientation_curr[0], self.orientation_curr[1], self.orientation_curr[2])
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
        twist = self.get_twist(linear,angular)
        self.jog_pub.publish(twist)



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