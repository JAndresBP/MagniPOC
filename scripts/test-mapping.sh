#!/bin/bash

# Fast local run of the mapping smoke test, without going through the full colcon test suite.
# Drives the robot itself (no rosbag fixture needed) -- see test_mapping_launch.py for the
# bag-replay variant. Runs the same test file CI exercises via
# magni_integration_tests/CMakeLists.txt + scripts/run-integration-test.sh.

source /opt/ros/jazzy/setup.bash
source install/setup.bash

launch_test src/magni_integration_tests/tests/test_mapping_smoke.py
