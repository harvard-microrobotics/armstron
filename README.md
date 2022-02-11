# Virtual Instron
This ROS package and associated GUI make use of the 6-axis force/torque sensor on the Universal Robot e-series to apply complex loads to things. We can essentially perform tests similar to an Instron Uniaxial Testing machine, but in all axis.


## Installation
1. Clone this package to the `src` folder of your catkin workspace
2. In the root folder of your workspace, install dependencies:
    - `rosdep install --from-paths src --ignore-src -r -y`
3. Build the workspace: `catkin_make`


## Usage

### Bringup the robot
1. _(Teach Pendant)_ Turn on the robot, get into _manual_ mode, then load the "EXTERNAL_CONTROL.urp" program.
2. _(Host Computer)_ `roslaunch ur_user_calibration bringup_armando.launch`
3. _(Teach Pendant)_ Run the "EXTERNAL_CONTROL.urp" program.

### Use the Virtual Instron test server
1. Start the test server: `roslaunch virtual_instron bringup_testing.launch`
2. Publish a goal message to the `/virtual_instron/goal` topic using rqt message publisher
```
    command: to_failure
    filename: "test.csv"
    params: {"test": {"motion": {"linear": [0.00, 0.0, 0.010], "angular": [0.0, 0.0, 0.0] }, "stop_conditions": {"max_position_z": 0.1 }},
             "preload": {"motion": {"linear": [0, 0, -0.0005], "angular": [0.0, 0, 0] }, "stop_conditions": {"max_force_z": 60 } }}

```
    - `params` _should be a json string._

### Start the GUI
_GUI Coming Soon!_


### Useful commands for debugging
- Show the controller manager: `rosrun rqt_controller_manager rqt_controller_manager`
- Enable sending of single messages
    - `rqt`
    - Go to _Plugins_ >> _Topics_ >> _Message Publisher_