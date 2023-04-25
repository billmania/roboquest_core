#!/usr/bin/env bash

cd /usr/src/ros2ws
source /opt/ros/humble/setup.bash

colcon build
source install/setup.bash

ros2 launch roboquest_core roboquest_core.launch.py
