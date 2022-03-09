# Armstron
This ROS package and associated GUI make use of the 6-axis force/torque sensor on the Universal Robot e-series to apply complex loads to things. We can essentially perform tests similar to an Instron Uniaxial Testing machine, but in all axis.


## Installation
1. Set up your robot arm for use with the [Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).
    - Clone the package to the **src** folder of your catkin workspace
    - Follow instructions in the [ur_robot_driver/doc](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/doc) folder. to set up the URCap on the robot
2. Clone this package (armstron) to the **src** folder of your catkin workspace
3. In the root folder of your workspace, install dependencies:
    - `rosdep install --from-paths src --ignore-src -r -y`
4. Navigate to the armstron package folder and install a few extra non-ROS python requirements:
    - `cd src/armstron/armstron`
    - `pip install -r requirements.txt`
5. Navigate back to the workspace folder, and build your workspace (`catkin_make`)


## Usage

### Bringup the robot
1. _(Teach Pendant)_ Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
2. _(Host Computer)_ `roslaunch ur_user_calibration bringup_armando.launch`
3. _(Teach Pendant)_ Run the "EXTERNAL_CONTROL.urp" program.

### Use the Armstron test server
1. Start the test server: `roslaunch armstron bringup_testing.launch`
2. Start a test by publishing a goal message to the `/armstron/goal` topic

```bash
    roslaunch armstron run_test.launch config:="ceti_pull_test.yaml" save:="~/vinst_data/testing_launch.csv"
    roslaunch armstron run_test.launch config:="ceti_force_hold.yaml" save:="~/vinst_data/testing_launch.csv"
```

### Start the GUI
1. Start the test server: `roslaunch armstron bringup_testing.launch`
2. Start the Armstron GUI:

```bash
    rosrun armstron gui.py
```


### Useful commands for debugging
- Show the controller manager: `rosrun rqt_controller_manager rqt_controller_manager`
- Enable sending of single messages
    - `rqt`
    - Go to _Plugins_ >> _Topics_ >> _Message Publisher_
