#!/usr/bin/env python
import time
#import roslib; roslib.load_manifest('ur_driver')
import rospy
import rospkg
import yaml
import ast
import os
import sys

from armstron.hardware_interface import DataLogger

folder = 'data'
filenames = ['test_data_0.csv','test_data_1.csv', 'test_data_2.csv']

def main():
    filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')
    filepath_data = os.path.join(rospkg.RosPack().get_path('armstron'), folder)

    config_file = os.path.join(filepath_config,'data_to_save.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    for filename in filenames:
        data_filename = os.path.join(filepath_data, filename)
        logger = DataLogger(data_filename,config)
        try:
            logger.start()
            rospy.sleep(0.5)
            logger.pause()
            rospy.sleep(1.0)
            logger.resume()
            rospy.sleep(0.5)
            logger.stop()
            logger.shutdown()
            

        except KeyboardInterrupt:
            print("V_INST TEST SERVER: Shutting Down")
            logger.shutdown()

        except rospy.ROSInterruptException:
            print("V_INST TEST SERVER: Shutting Down")
            logger.shutdown()


if __name__ == '__main__':
    rospy.init_node('v_inst_test_datalogger', disable_signals=True)
    print("V_INST DATALOGGER: Node Initiatilized (%s)"%(rospy.get_name()))
    main()

    

    

