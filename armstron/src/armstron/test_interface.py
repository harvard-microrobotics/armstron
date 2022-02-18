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

class TestRunner():
    '''
    A ROS Action server to run single tests.

    Parameters
    ----------
    name : str
        Name of the Action Server
    '''

    def __init__(self, name, debug=False):

        self.DEBUG = debug
        self._action_name = name

        # Make an action client
        self._test_client = actionlib.SimpleActionClient(self._action_name, msg.RunTestAction)
        self._test_client.wait_for_server()


    def set_profile(self, profile):
        '''
        Set the config profile
        '''
        self.config = profile


    def set_savefile(self, save_file):
        '''
        Set the filename to save
        '''
        self.save_file = save_file


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
        self._test_client.cancel_all_goals()