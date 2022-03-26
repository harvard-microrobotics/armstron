.. _first_test:

===================
Run a Test Manually
===================

Lets walk through the manual way to build and run a test.


.. contents:: Contents:
    :local:
    :depth: 1



Build a Test Profile
____________________

Test profiles are defined in YAML files inside the `armstron/config/test_profiles <https://github.com/harvard-microrobotics/armstron/tree/main/armstron/config/test_profiles>`_ folder.


Profile Structure
-----------------

Test profiles must be structured in a specific way:

1. **type** (``str``): The type of profile. For now, the only valid value is "*sequence*"
2. **params** (``dict``): A set of parameters

    a. **preload** (``list``): A list of actions to perform during the "preload" phase
    b. **test** (``list``): A list of actions to perform during the "test" phase


Testing Actions
---------------

Actions can be either "jog", "pose" or "balance".

**Jog** steps have motion parameters (how the arm moves) and stop conditions (when the arm should stop moving):

.. code-block:: yaml

    jog: # jog motions about the end effector (TCP of the robot)
        linear: [X, Y, Z]          # [mm/sec]
        angular: [X, Y, Z]         # [rad/sec]
    stop_conditions:
        max_time: [TIME]           # [sec]
        max_force_x: [FORCE]       # [N], options: min/max, and x/y/z
        max_torque_x: [TORQUE]     # [Nm], options: min/max, and x/y/z
        max_position_x: [POSITION] # [m], options: min/max, and x/y/z
        max_orientation_x: [ORI]   # [rad], options: min/max, and x/y/z

**Pose** steps move the arm to a specific pose over a given time:

.. code-block:: yaml

    pose: # pose of the end effector (TCP of the robot)
        position: [X, Y, Z]          # [m]
        orientation: [X, Y, Z]         # [degrees (euler angles)]
    stop_conditions:
        max_time: [TIME]           # [sec]
        max_force_x: [FORCE]       # [N], options: min/max, and x/y/z
        max_torque_x: [TORQUE]     # [Nm], options: min/max, and x/y/z
        max_position_x: [POSITION] # [m], options: min/max, and x/y/z
        max_orientation_x: [ORI]   # [rad], options: min/max, and x/y/z

**Balancing** steps can balance (zero) either the pose or the F/T sensor readings:

.. code-block:: yaml

    balance: [TYPE]   # options: 'pose' and 'ft'

*Note:* Omit (or comment out) ``stop_conditions`` if you do not want to use them.


Examples
--------

Here is an example of a simple testing profile:

:fa:`file-code-o,fa-primary` `ceti_pull_test.yaml <https://github.com/harvard-microrobotics/armstron/blob/main/armstron/config/test_profiles/ceti_pull_test.yaml>`_


.. literalinclude:: ../../armstron/config/test_profiles/ceti_pull_test.yaml


Here is an example of a more complex, multi-step testing profile:

:fa:`file-code-o,fa-primary` `ceti_force_hold.yaml <https://github.com/harvard-microrobotics/armstron/blob/main/armstron/config/test_profiles/ceti_force_hold.yaml>`_


.. literalinclude:: ../../armstron/config/test_profiles/ceti_force_hold.yaml




Run a Test
__________

Running a test is just a matter of running one terminal command (after you start the rest of the system up) with the profile you want to use, and the filename to save data.

Testing Procedure
-----------------

1. Bringup the robot

    a. *(Teach Pendant)* Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
    b. *(Host Computer)* In a new terminal: ``roslaunch ur_user_calibration bringup_armando.launch``
    c. *(Teach Pendant)* Run the "EXTERNAL_CONTROL.urp" program.

2. Start the Armstron test server (this waits for tests to be started, and handles balancing and estop commands)

    a. In a new terminal, start the test server: ``roslaunch armstron bringup_testing.launch``


3. Start a test (for example, *ceti_pull_test.yaml*), and save data in the Documents folder (*~/Documents/vinst_data/test.csv*)
    
    a. In a new terminal, run:

.. code-block:: bash

    roslaunch armstron run_test.launch config:="ceti_pull_test.yaml" save:="~/Documents/vinst_data/test.csv"

.. note::
    
    If you want to run more tests, just keep repeating step 3. Savefile names are auto-incremented to prevent overwriting of data, so you can keep sending the same filename (and thus the same terminal command) over and over to keep repeating the same test procedure.


More Details
------------

When running the test, the launch file you are calling takes care of routing parameters to the correct script:

:fa:`file-code-o,fa-primary` `run_test.launch <https://github.com/harvard-microrobotics/armstron/blob/main/armstron/launch/run_test.launch>`_


.. literalinclude:: ../../armstron/launch/run_test.launch



This launch file invokes a ros node that creates a :ref:`TestRunner <test_interface_api>` object:

:fa:`file-code-o,fa-primary` `run_single_test.py <https://github.com/harvard-microrobotics/armstron/blob/main/armstron/scripts/run_single_test.py>`_


.. literalinclude:: ../../armstron/scripts/run_single_test.py