#!/usr/bin/env bash

cd /usr/src/ros2ws
source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch roboquest_core servo_tester.launch.py
