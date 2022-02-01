from __future__ import print_function
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib

from hand_arm_cbt.arm_moveit import MoveItPythonInteface as ur_traj_sender_moveit

class RobotController:
    def __init__(self, traj=None):