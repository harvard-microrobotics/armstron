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
from armstron.utils import load_yaml
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


    def load_profile(self, filename):
        '''
        Load the config profile from a file

        Parameters
        ----------
        filename : str
            Filename to load
        '''
        profile = load_yaml(filename)
        self.set_profile(profile)


    def set_profile(self, profile):
        '''
        Set the config profile

        Parameters
        ----------
        profile : dict
            Testing profile to use
        '''
        self.config = profile


    def set_savefile(self, save_file):
        '''
        Set the filename to save

        Parameters
        ----------
        save_file : str
            Filename to save data to
        '''
        self.save_file = save_file


    def get_test_status(self):
        '''
        Get the current state of the test

        Returns
        -------
        state : ActionFeedback
            The current state of the test
        '''
        return self._test_client.get_state()


    def run_test(self, wait_for_finish=True):
        '''
        Run a test and wait for the result

        Parameters
        ----------
        wait_for_finish : bool
            Wait for the test to finish

        Returns
        -------
        result : ActionResult
            The result of the test.
        '''
        # Build the goal message to send
        goal = msg.RunTestGoal()
        goal.command = self.config['type']
        goal.params = json.dumps(self.config['params'])
        goal.filename = self.save_file

        # Tell the test server to run the test.
        self._test_client.send_goal(goal)

        if wait_for_finish:
            # Wait for the server to finish performing the test.
            self._test_client.wait_for_result()
            # Return the result
            return self._test_client.get_result() 
        
        else:
            return True


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