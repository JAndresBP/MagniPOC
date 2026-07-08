#!/bin/bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --base-paths \
    src/ubiquity_motor_ros2 \
    src/ubiquity_motor_ros2/ubiquity_motor_ros2_msgs \
    src/magni_description \
    src/franka_description \
    src/rebotarm_ros2 \
    src/magni_bringup \
    src/magni_mapping \
    src/magni_webots \
    src/magni_control_station \
    --cmake-args --event-handlers console_direct+