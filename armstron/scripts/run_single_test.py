#!/usr/bin/env python
import time
#import roslib; roslib.load_manifest('ur_driver')
import rospy
import rospkg
import actionlib
import yaml
import json
import ast
import os
import sys
import copy

import armstron.msg as msg
from armstron.srv import Balance, Estop

filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')

class TestRunner():
    '''
    A ROS Action server to run single tests.

    Parameters
    ----------
    name : str
        Name of the Action Server
    '''

    def __init__(self, name):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self._action_name = rospy.get_param(rospy.get_name()+"/action_name",name)
        self.config_file = rospy.get_param(rospy.get_name()+"/config_file",None)
        self.save_file = rospy.get_param(rospy.get_name()+"/save_file",None)
        self.config_path = os.path.join(filepath_config,'test_profiles')
        self.config = self._get_config(os.path.join(self.config_path, self.config_file))

        # Make an action client
        self._test_client = actionlib.SimpleActionClient(self._action_name, msg.RunTestAction)
        self._test_client.wait_for_server()

        # Make a service proxy for the estop service


    def _get_config(self, filename):
        '''
        Load a testing profile (config)

        Parameters
        ----------
        filename : str or Path
            Filename to load config from

        Returns
        -------
        config : dict
            The test config
        '''
        with open(filename, 'r') as f:
            config=yaml.safe_load(f)
        return config


    def run_test(self):
        '''
        Run a test and wait for the result

        Returns
        -------
        result : ActionResult
            The result of the test.
        '''
        # Balance at start
        balance = self.config['balance'].lower()
        if balance!="":
            if 'pose' in balance:
                self.balance('pose')
            if 'ft' in balance:
                self.balance('ft')

        # Build the goal message to send
        goal = msg.RunTestGoal()
        goal.command = self.config['type']
        goal.params = json.dumps(self.config['params'])
        goal.filename = self.save_file

        # Tell the test server to run the test.
        self._test_client.send_goal(goal)

        # Wait for the server to finish performing the test.
        self._test_client.wait_for_result()

        # Return the result
        return self._test_client.get_result() 


    def estop(self):
        '''
        Perform an Emergency Stop
        '''
        # Send estop service call
        self._call_service(self._action_name+'/estop',Estop)


    def balance(self, type):
        '''
        Balance either the load or F/T sensor

        Parameters
        ----------
        type : str
            The type of balance to perform. Must be either
            ``pose`` or ``ft``
        '''
        # Send estop service call
        self._call_service(self._action_name+'/balance',Balance, type = str(type))


    def _call_service(self, name, service_type, **fargs):
        '''
        Call a service and handle the result

        Parameters
        ----------
        name : str
            Name of the service to call
        service_type : ROS srv
            ROS service class (must be correct for the service name you are calling)
        **args : Any
            Arguments to pass to the service

        Returns
        -------
        response : ROS srv response
            The response from the service. Returns ``None`` if the service
            call was unsuccessful.
        '''
        rospy.wait_for_service(name)
        try:
            fun = rospy.ServiceProxy(name, service_type)
            resp1 = fun(**fargs)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None


    def shutdown(self):
        '''
        Shut down the node gracefully
        '''
        pass


        
if __name__ == '__main__':
    try:
        rospy.init_node('v_inst_test_runner', disable_signals=True)
        print("V_INST TEST RUNNER: Node Initiatilized (%s)"%(rospy.get_name()))
        sender = TestRunner(rospy.get_name())
        sender.run_test()
        print("V_INST TEST RUNNER: Ready!")
        sender.shutdown()

    except KeyboardInterrupt:
        print("V_INST TEST RUNNER: Shutting Down")
        sender.estop()
        sender.shutdown()

    except rospy.ROSInterruptException:
        print("V_INST TEST RUNNER: Shutting Down")
        sender.estop()
        sender.shutdown()