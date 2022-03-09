#!/usr/bin/env python
import rospy
import rospkg
import os

from armstron.test_interface import TestRunner

filepath_config = os.path.join(rospkg.RosPack().get_path('armstron'), 'config')  
   
if __name__ == '__main__':
    try:
        rospy.init_node('v_inst_test_runner', disable_signals=True)
        print("V_INST TEST RUNNER: Node Initiatilized (%s)"%(rospy.get_name()))

        debug = rospy.get_param(rospy.get_name()+"/DEBUG",False)
        action_name = rospy.get_param(rospy.get_name()+"/action_name",None)
        config_file = rospy.get_param(rospy.get_name()+"/config_file",None)
        save_file = rospy.get_param(rospy.get_name()+"/save_file",None)

        config_path = os.path.join(filepath_config,'test_profiles',config_file)

        # Set settings
        sender = TestRunner(action_name, debug=debug)
        sender.load_profile(config_path)
        sender.set_savefile(save_file)

        print("V_INST TEST RUNNER: Running Test")
        sender.run_test(wait_for_finish=True)
        print("V_INST TEST RUNNER: Finished!")
        sender.shutdown()

    except KeyboardInterrupt:
        print("V_INST TEST RUNNER: Shutting Down")
        sender.estop()
        sender.shutdown()

    except rospy.ROSInterruptException:
        print("V_INST TEST RUNNER: Shutting Down")
        sender.estop()
        sender.shutdown()