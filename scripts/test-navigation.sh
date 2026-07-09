#!/bin/bash

# Fast local run of the Nav2 navigation smoke test, without going through the full colcon test
# suite. Runs the same test file CI exercises via magni_integration_tests/CMakeLists.txt +
# scripts/run-integration-test.sh.

source /opt/ros/jazzy/setup.bash
source install/setup.bash

launch_test src/magni_integration_tests/tests/test_navigation_smoke.py
