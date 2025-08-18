#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

ros2 launch magni_webots magni_spawn.launch.py
