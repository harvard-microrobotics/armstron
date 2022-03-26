# Armstron
This ROS package and associated GUI make use of the 6-axis force/torque sensor on the Universal Robot e-series to apply complex loads to things. We can essentially perform tests similar to an Instron Uniaxial Testing machine, but in all axes.


## Installation
1. Set up your robot arm for use with the [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).
    - Clone the package to the **src** folder of your catkin workspace
    - Follow instructions in the [ur_robot_driver/doc](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/doc) folder.
    - Be sure to create a calibration package for your robot per [these instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_calibration). In this example, we have created a package called "_ur_user_calibration_" with a dedicated launch file "_bringup_armando.launch_" where the IP address of the robot is set.
2. Set up this package
    1. Clone this package (armstron) to the **src** folder of your catkin workspace
    2. In the root folder of your workspace, install dependencies:
        - `rosdep install --from-paths src --ignore-src -r -y`
    3. Navigate to the armstron package folder and install a few extra non-ROS python requirements:
        - `cd src/armstron/armstron`
        - `pip install -r requirements.txt`
    4. Navigate back to the workspace folder, and build your workspace (`catkin_make`)
3. Set up the runfile (so you can run Armstron as though it is a regular application)
    1. Navigate to the armstron top level folder:
        - `cd src/armstron`
    2. Generate the run files
        - `sudo bash armstron/bash/generate_desktop.sh [ROBOT PACKAGE] [ROBOT LAUNCH FILE]`
        - For example: `sudo bash armstron/bash/generate_desktop.sh ur_user_calibration bringup_armando.launch`
    3. Now you can run Armstron with one click by going to your application menu (Super + A) and choosing "Armstron".


## Usage

### Bringup the robot
1. _(Teach Pendant)_ Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
2. _(Teach Pendant)_ Start the robot (tap the small red dot on the bottom left corner)

### Start Armstron
1. _(Host Computer)_ Choose "Armstron" from your application menu (Super + A).
    - This starts communication with the robot arm, starts the Armstron test server, and starts the Armstron GUI.
2. Use the GUI to load/build test profiles, set data save locations, and run tests.


## Advanced Usage

Since everything is modular, you can run each part of the Armstron software stack as independent ROS processes. This is useful for debugging purposes

### Bringup the robot
1. _(Teach Pendant)_ Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
2. _(Teach Pendant)_ Start the robot (tap the small red dot on the bottom left corner)
3. _(Host Computer)_ (new terminal): `roslaunch ur_user_calibration bringup_armando.launch`
4. _(Teach Pendant)_ Run the "EXTERNAL_CONTROL.urp" program.

### Use the Armstron test server
1. Start the test server (new terminal): `roslaunch armstron bringup_testing.launch`
2. Start a test (new terminal):

```bash
    roslaunch armstron run_test.launch config:="ceti_pull_test.yaml" save:="~/vinst_data/testing_launch.csv"
    roslaunch armstron run_test.launch config:="ceti_force_hold.yaml" save:="~/vinst_data/testing_launch.csv"
```

### Start the GUI
1. Start the test server (new terminal): `roslaunch armstron bringup_testing.launch`
2. Start the Armstron GUI (new terminal): `rosrun armstron gui.py`


### Useful commands for debugging
- Show the controller manager: `rosrun rqt_controller_manager rqt_controller_manager`
- Enable sending of single messages
    - `rqt`
    - Go to _Plugins_ >> _Topics_ >> _Message Publisher_
