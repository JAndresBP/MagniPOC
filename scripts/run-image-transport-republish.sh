#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/ws/install/setup.bash

ros2 run image_transport republish --ros-args -p in_transport:=raw -p out_transport:=compressed --remap in:=/rpicam/image_raw --remap out/compressed:=/rpicam/image_raw/compressed