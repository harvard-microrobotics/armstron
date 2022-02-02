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
        self.save_filename = filename
        self.poll_rate = params.get('poll_rate', 100)
        self.motion_mode = params['mode']
        self.preload = params.get('preload', None)
        self.motion_direction = params['motion']
        self.stop_conditions = params['stop_conditions']
        self.robot = RobotController()
        self.jogpub = rospy.Publisher('twist_controller/command', Twist, queue_size=10)


    def validate_params(self, params):
        if not isinstance(params, dict):
            print("Params must be a dict")
            return False
        
        keys = params.keys()
        if ('motion' not in keys) or ('mode' not in keys) or ('stop_conditions' not in keys):
            print(params.keys())
            return False
        else:
            return True
   
    def run_preload(self):
        if self.preload is None:
            return True

        if self.preload.get("duration", None):

            r = rospy.Rate(self.poll_rate)
            self._set_jog([0,0,-0.01], [0,0,0])
            
            # Apply desired preload force
            preload_duration = self.preload.get("duration")
            run_flag = True
            start_time = rospy.get_rostime().to_sec()
            curr_time = rospy.get_rostime().to_sec()
            i=0
            while curr_time-start_time<=preload_duration:
                curr_time = rospy.get_rostime().to_sec()

                print(i)

                i+=1   
                r.sleep()
            
            self._set_jog([0,0,0], [0,0,0])


    def run_test(self):
        print(self.params['motion']['linear'],self.params['motion']['angular'])

        self._set_jog(self.params['motion']['linear'], self.params['motion']['angular'])

        time.sleep(1.0)

        self._set_jog([0,0,0], [0,0,0])

        


    def run(self):
        # Switch controller to jog control:
        self.robot.set_controller('twist_controller')
        time.sleep(0.5)

        inp = raw_input('Start Preload? Press [ENTER] ')

        self.run_preload()
        time.sleep(0.5)

        inp = raw_input('Start Test? Press [ENTER] ')

        self.run_test()

    
    
    def _set_jog(self, linear, angular):
        twist = self.robot.get_twist(linear,angular)
        self.jogpub.publish(twist)


    def shutdown(self):
        self._set_jog([0,0,0], [0,0,0])