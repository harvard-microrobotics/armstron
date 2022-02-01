#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import rospkg
import actionlib
import yaml
import os
import sys


#Import the specific messages that we created.
import virtual_instron.msg as msg
from hand_arm_cbt.arm_moveit import MoveItPythonInteface as ur_traj_sender_moveit
from geometry_msgs import Wrench


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

filepath_config = os.path.join(rospkg.RosPack().get_path('virtual_instron_gui'), 'config')

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

        # Get planning interface and update config
        self.arm_sender = ur_traj_sender_moveit(joint_names=JOINT_NAMES, non_excecuting=False)
        self.arm_sender.config_planner(os.path.join(filepath_config,'moveit_config.yaml'))

        # Subscribe to the wrench topic
        rospy.Subscriber('/wrench', Wrench, self.update_wrench)


        # Start an actionlib server
        self._feedback.success = False
        self._feedback.status = "Idle"
        self._result.success = False
        self._as = actionlib.SimpleActionServer(self._action_name, msg.RunTestAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()


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
        r = rospy.Rate(3000)
        
        # Initiatilize the feedback
        self._feedback.success = False
        self._feedback.status = "Starting"

        goal_type = self.validate_goal(goal.command)

        if goal_type is None:

            self._feedback.success = False
            self._feedback.status = "Invalid test type"
            self._as.publish_feedback(self._feedback)


        else:    
            # start executing the test
            cmd_str = self.send_command(goal.command, goal.args)

            self._feedback.status = "Testing"
            self._as.publish_feedback(self._feedback)

            i =0
            while i<1000 and not rospy.is_shutdown() and not self._as.is_preempt_requested():
                i+=1
                r.sleep()

                
            self._feedback.success = True
            self._feedback.status = "Complete"
            self._as.publish_feedback(self._feedback)
    
        
        
          
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
        elif 'cyclic' in goal.command:
            goal_type = goal.command
        else:
            goal_type = None

        return goal_type


    def update_wrench(self,data):
        '''
        Update the internal value of the wrench.

        Parameters
        ----------
        data : geometry_msgs/Wrench
            Wrench message
        '''
        self.force_curr = [data.force.x, data.force.y, data.force.z]
        self.torque_curr = [data.torque.x, data.torque.y, data.torque.z]




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

    