#!/bin/bash

source /opt/ros/jazzy/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='ps3' publish_stamped_twist:=true config_filepath:=/home/ws/install/magni_bringup/share/magni_bringup/config/joy.config.yaml