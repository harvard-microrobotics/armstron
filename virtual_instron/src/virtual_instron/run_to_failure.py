from __future__ import print_function
import os
import time
import yaml
import rospy
import rospkg
import actionlib

from virtual_instron.hardware_interface import DataLogger
import virtual_instron.utils as utils
from geometry_msgs.msg import Twist, Vector3

class RunTest:
    def __init__(self, filename, robot, params={}):
        if not self.validate_params(params):
            print("Invalid parameter set. Skipping test")
            return

        self.params = params
        self.poll_rate = params.get('poll_rate', 100)
        self.motion_mode = params['mode']
        self.preload = params.get('preload', None)
        self.motion_direction = params['motion']
        self.global_offsets = params['offsets']
        self.stop_conditions = params['stop_conditions']
        self.robot = robot
        self.jogpub = rospy.Publisher('twist_controller/command', Twist, queue_size=10)
        self.logger = self.create_logger(filename)


    def create_logger(self,filename):
        filepath_config = os.path.join(rospkg.RosPack().get_path('virtual_instron'), 'config')
        log_config_file = os.path.join(filepath_config,'data_to_save.yaml')
        with open(log_config_file, 'r') as f:
            log_config = yaml.safe_load(f)
        
        logger = DataLogger(filename,log_config)
        return logger


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
   

    def get_condition_functions(self, stop_conditions, condition_values):

        idx_map = {'x':0, 'y':1, 'z':2, 'w':3}

        def get_position(idx):
            print(self.robot.position_curr[idx])
            return self.robot.position_curr[idx]

        def get_orientation(idx):
            return self.robot.orientation_curr[idx]

        def get_force(idx):
            print(self.robot.force_curr[idx])
            return self.robot.force_curr[idx]
        
        def get_torque(idx):
            return self.robot.torque_curr[idx]

        def get_time():
            print(rospy.get_rostime().to_sec() - self.start_time)
            return rospy.get_rostime().to_sec() - self.start_time

        function_list = []
        for condition, val in zip(stop_conditions, condition_values):
                       
            if 'position' in condition:
                idx = condition.split('position_')[1]
                fun = lambda : get_position(idx_map[idx])
            elif 'orientation' in condition:
                idx = condition.split('orientation_')[1]
                fun = lambda : get_orientation(idx_map[idx])
            elif 'force' in condition:
                idx = condition.split('force_')[1]
                fun = lambda : get_force(idx_map[idx])
            elif 'torque' in condition:
                idx = condition.split('torque_')[1]
                fun = lambda : get_torque(idx_map[idx])
            elif 'time' in condition:
                fun = lambda : get_time()
            else:
                fun = lambda : True

            if 'max' in condition:
                function_list.append(lambda : fun() > val)
            elif 'min' in condition:
                function_list.append(lambda : fun() < val )
            else:
                function_list.append(lambda : fun())

        return function_list



    def run_preload(self):
        if self.preload is None:
            return True

        # Apply desired preload force
        preload_stop_conditions = [[],[]]
        for key in self.preload:
            preload_stop_conditions[0].append(key)
            preload_stop_conditions[1].append(self.preload[key])

        self.start_time = rospy.get_rostime().to_sec()
        condition_funs = self.get_condition_functions(preload_stop_conditions[0], preload_stop_conditions[1])


        run_flag = True
        preload_stop = False
        r = rospy.Rate(self.poll_rate)

        self._set_jog(self.params['preload_motion']['linear'], self.params['preload_motion']['angular'])
        i=0
        while not preload_stop:              
            i+=1   
            r.sleep()
            checks = []
            for condition in condition_funs:
                checks.append(condition())

            print(checks)
            preload_stop=utils.check_any(checks)
            
        self._set_jog([0,0,0], [0,0,0])


    def run_test(self):


        self._set_jog(self.params['motion']['linear'], self.params['motion']['angular'])

        time.sleep(1.0)

        self._set_jog([0,0,0], [0,0,0])

        


    def run(self):
        # Switch controller to jog control:
        self.robot.set_controller('twist_controller')
        time.sleep(0.5)

        inp = raw_input('Start Preload? Press [ENTER] ')

        
        self.logger.start()
        self.run_preload()
        self.logger.pause()  

        inp = raw_input('Start Test? Press [ENTER] ')

        self.logger.resume() 

        self.run_test()
        self.logger.stop()       

    
    
    def _set_jog(self, linear, angular):
        twist = self.robot.get_twist(linear,angular)
        self.jogpub.publish(twist)


    def shutdown(self):
        self._set_jog([0,0,0], [0,0,0])
        self.logger.shutdown()