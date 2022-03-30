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
import threading


#Import the specific messages that we created.
import armstron.msg as msg
from armstron.srv import Balance, BalanceResponse, Estop, EstopResponse
from armstron.hardware_interface import RobotController

filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')


class TestServer():
    '''
    A ROS Action server to run single tests.

    Parameters
    ----------
    name : str
        Name of the Action Server
    '''
    # create messages that are used to publish feedback/result
    _feedback = msg.RunTestFeedback()
    _result = msg.RunTestResult()

    def __init__(self, name):

        self.DEBUG = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        self._action_name = rospy.get_param(rospy.get_name()+"/action_name",name)

        self.robot = RobotController()
        self.robot.set_offsets(None)


        # Start an actionlib server
        self._feedback.success = False
        self._feedback.status = "Idle"
        self._result.success = False
        self._as = actionlib.SimpleActionServer(self._action_name, msg.RunTestAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.balance_service = rospy.Service(self._action_name+'/balance', Balance, self.balance)
        self.estop_service = rospy.Service(self._action_name+'/estop', Estop, self.estop)
        self.estop_state=False


    def balance(self,data):
        if data.type == 'pose':
            self.balance_pose()
            success= True
        elif data.type == 'ft':
            self.balance_ft()
            success=True
        else:
            success = False

        return BalanceResponse(success)

    def estop(self,data):
        self.trigger_estop()
        success=True
        return EstopResponse(success)

    def trigger_estop(self):
        self.estop_state = True

    def reset_estop(self):
        self.estop_state = False

    
    def balance_pose(self):
        self.robot.balance_pose()


    def balance_ft(self):
        self.robot.balance_ft()


    def execute_cb(self, goal):
        '''
        Run a test when a goal is sent

        Parameters
        ----------
        goal : RunTrajGoal
            Action goal with associated parameters for the test to be done
        '''
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()

        #rospy.loginfo("Start: %s"%(goal.command))

        # helper variables
        r = rospy.Rate(500)
        
        # Initiatilize the feedback
        self._feedback.success = False
        self._feedback.status = "Starting"

        RunTest = self.validate_goal(goal)

        if RunTest is None:

            self._feedback.success = False
            self._feedback.status = "Invalid test type"
            self._as.publish_feedback(self._feedback)


        else:
            self.reset_estop()
            try:
                param_dict = ast.literal_eval(goal.params)
                if param_dict.get('offsets', None) is None:
                    param_dict['offsets'] = None
                
                print(param_dict.keys())
                test_runner = RunTest(os.path.expanduser(goal.filename), self.robot, self._as, param_dict)

            except:
                raise

            try:
                self._feedback.status = "Testing"
                self._as.publish_feedback(self._feedback)

                def worker(kill_now):
                    #rospy.init_node('v_inst_test_runner', disable_signals=True)
                    success = test_runner.run(kill_now)
                    if success:
                        self.success.set()
                    else:
                        self.success.clear()

                # Start a new process to run the test, then wait for it to finish (or estop)
                #manager = mp.Manager()
                #success_dict = manager.dict()
                #success_dict['success'] = False
                self.success = threading.Event()
                self.success.clear()
                kill_now = threading.Event()
                kill_now.clear()
                p = threading.Thread(target=worker, args=(kill_now,))
                p.start()            

                while (not self.success.is_set()) and (not self.estop_state):
                    if p.is_alive():
                        print("Alive")
                        p.join(1.0) # Join for 1 second,
                    else:
                        self.trigger_estop()
                        print("Finished")
                        

                # If the process is still alive, terminate it.
                self.reset_estop()
                if p.is_alive():
                    kill_now.set()
                    p.join()
          
                self._feedback.success = self.success.is_set()
                self._as.publish_feedback(self._feedback)
            
            except:
                test_runner.shutdown()
                raise

        
        self._result.success = self._feedback.success
        self._as.set_succeeded(self._result)

        if self.DEBUG:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            rospy.loginfo("End: %s"%(goal.command))


    def validate_goal(self, goal):
        '''
        Validate the input goal to make sure it's a valid command

        Parameters
        ----------
        goal : RunTrajGoal
            The action goal you want to validate
        
        Returns
        -------
        goal_type : str
            The type of test to be performed. If invalid, ``goal_type= None``
        '''
        if 'to_failure' in goal.command:
            goal_type = goal.command
            from armstron.run_to_failure import RunTest

        elif 'sequence' in goal.command:
            goal_type = goal.command
            from armstron.run_sequence import RunTest

        elif 'cyclic' in goal.command:
            goal_type = goal.command
            from armstron.run_cyclic import RunTest

        elif 'static' in goal.command:
            goal_type = goal.command
            from armstron.run_static import RunTest

        else:
            RunTest = None
        return RunTest



    def shutdown(self):
        '''
        Shut down the node gracefully
        '''
        pass


        
if __name__ == '__main__':
    try:
        rospy.init_node('v_inst_test_server', disable_signals=True)
        print("V_INST TEST SERVER: Node Initiatilized (%s)"%(rospy.get_name()))
        server = TestServer(rospy.get_name())
        print("V_INST TEST SERVER: Ready!")
        rospy.spin()

    except KeyboardInterrupt:
        print("V_INST TEST SERVER: Shutting Down")
        server.shutdown()

    except rospy.ROSInterruptException:
        print("V_INST TEST SERVER: Shutting Down")
        server.shutdown()

    