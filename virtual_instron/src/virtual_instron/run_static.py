from __future__ import print_function
import time
import rospy
import actionlib

from virtual_instron.hardware_interface import RobotController

class RunTest:
    def __init__(self, filename, params={}):
        self.params = params
        self.robot = RobotController()
   
    def run(self):
        # Test switching controllers:
         self.robot.set_controller('scaled_pos_joint_traj_controller')
