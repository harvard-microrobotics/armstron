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


#Import the specific messages that we created.
import virtual_instron.msg as msg
from virtual_instron.hardware_interface import RobotController

filepath_config = os.path.join(rospkg.RosPack().get_path('virtual_instron'), 'config')


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

    
    def balance_pose(self):
        offset = self.robot.get_offsets()
        offset['position'] = self.robot.position_raw
        offset['orientation'] = self.robot.orientation_raw
        self.robot.set_offsets(offset)


    def balance_ft(self):
        offset = self.robot.get_offsets()
        offset['force'] = self.robot.force_raw
        offset['torque'] = self.robot.torque_raw
        self.robot.set_offsets(offset)


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

        # Check if we should balance the signals
        if '_balance' in goal.command:
            if 'pose' in goal.command:
                self.balance_pose()
                rospy.loginfo("Robot Pose Balanced")
            elif 'ft' in goal.command:
                self.balance_ft()
                rospy.loginfo("F/T Sensor Balanced")
            self._as.set_succeeded(self._result)
            return



        RunTest = self.validate_goal(goal)

        if RunTest is None:

            self._feedback.success = False
            self._feedback.status = "Invalid test type"
            self._as.publish_feedback(self._feedback)


        else:
            try:
                param_dict = ast.literal_eval(goal.params)
                if param_dict.get('offsets', None) is None:
                    param_dict['offsets'] = None
                test_runner = RunTest(os.path.expanduser(goal.filename), self.robot, param_dict)
                test_runner.run()
                test_runner.shutdown()

                self._feedback.status = "Testing"
                self._as.publish_feedback(self._feedback)

                #i =0
                #while i<1000 and not rospy.is_shutdown() and not self._as.is_preempt_requested():
                #    print(test_runner.robot.force_curr)
                #    i+=1
                #    r.sleep()

                    
                self._feedback.success = True
                self._feedback.status = "Complete"
                self._as.publish_feedback(self._feedback)
            
            except:
                test_runner.shutdown()
                raise
    
        
        
          
        if self._feedback.success:
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
            from virtual_instron.run_to_failure import RunTest

        elif 'cyclic' in goal.command:
            goal_type = goal.command
            from virtual_instron.run_cyclic import RunTest

        elif 'static' in goal.command:
            goal_type = goal.command
            from virtual_instron.run_static import RunTest

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

    