#!/bin/bash

# Source the catkin workspace that this package is installed in
directory=/home/armando/Documents/armando_ws_update/src/armstron
base=$(basename $directory)
finished=false
echo $base
while [ $finished != true ]
do
    directory=$(dirname $directory)
    base=$(basename $directory)
    echo $base

    if [ $base == "src" ]; then
        finished=true
    fi

    if [ $base == '/' ]; then
        break;
    fi

done

if [ $finished != true ]; then
    echo "Catkin workspace not found"
    sleep 5.0
    exit 1
fi

directory=$(dirname $directory)

source "$directory/devel/setup.bash"
echo "Sourced: $directory"

# Start the robot arm
roslaunch ur_user_calibration bringup_armando.launch &
robot_pid=$!
sleep 3

# Start the test server
roslaunch armstron bringup_testing.launch &
armstron_pid=$!
sleep 2

# Start the Armstron GUI
rosrun armstron gui.py &
gui_pid=$!

# Wait until the gui is closed
gui_running=true
while $gui_running
do
    if ps $gui_pid > /dev/null ; then
        gui_running=true
    else 
        echo -n "GUI ended"
        gui_running=false
    fi
    sleep 0.5
done

# Stop the robot and test processes
kill -INT $robot_pid
kill -INT $armstron_pid
sleep 1

# Wait until everything is stopped, then close
wait $(jobs -p)