==================
Hardware Interface
==================



Robot Controller
------------------

Control the robot's motion and get data. This interface relies on the `Universal_Robots_ROS_Driver <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>`_ package to control UR robots using the builtin ROS controllers.


.. autoclass:: hardware_interface.RobotController
   :members: 
   :exclude-members: mute



Data Logger
-----------------------------

	Log data to syncronized files. Given a filename and a map of topics (see `armstron/config/data_to_save.yaml <https://github.com/harvard-microrobotics/armstron/blob/main/armstron/config/data_to_save.yaml>`_), log data from ROS topics to CSV files.

.. autoclass:: hardware_interface.DataLogger
   :members: 
   :exclude-members: mute