from __future__ import print_function
import time
import rospy
import actionlib

from virtual_instron.hardware_interface import RobotController
from geometry_msgs.msg import Twist, Vector3

class RunTest:
    def __init__(self, filename, params={}):
        if not self.validate_params(params):
            print("Invalid parameter set. Skipping test")
            return
        self.params = params
        self.robot = RobotController()
        self.jogpub = rospy.Publisher('twist_controller/command', Twist, queue_size=10)


    def validate_params(self, params):
        if not isinstance(params, dict):
            print("Params must be a dict")
            return False
        
        keys = params.keys()
        if ('motion' not in keys) or ('mode' not in keys):
            print(params.keys())
            return False
        else:
            return True
   
   
    def run(self):
        # Switch controller to jog control:
        self.robot.set_controller('twist_controller')
        time.sleep(0.5)

        print(self.params['motion']['linear'],self.params['motion']['angular'])
        self._set_jog(self.params['motion']['linear'], self.params['motion']['angular'])
        time.sleep(1.0)
        self._set_jog([0,0,0], [0,0,0])

    
    
    def _set_jog(self, linear, angular):
        twist = self.robot.get_twist(linear,angular)
        self.jogpub.publish(twist)
