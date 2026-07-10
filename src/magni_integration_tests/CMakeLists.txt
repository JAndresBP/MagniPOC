cmake_minimum_required(VERSION 3.8)
project(magni_integration_tests)

# Needed explicitly for a from-scratch build (ament_package() below requires it) -- previously
# only found transitively via ament_cmake_ros inside if(BUILD_TESTING), which happened to mask
# this on incremental builds that reused an existing CMake cache.
find_package(ament_cmake REQUIRED)

########
# test #
########

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  # Register integration tests
  add_ros_isolated_launch_test(tests/test_mapping_launch.py)
  add_ros_isolated_launch_test(tests/test_mapping_smoke.py)
  add_ros_isolated_launch_test(tests/test_speed_response.py)
  add_ros_isolated_launch_test(tests/test_navigation_smoke.py)
  # Longer path around a static obstacle takes 75-100s in practice -- default ctest TIMEOUT (60s)
  # kills it before it can write a result, so bump it explicitly.
  add_ros_isolated_launch_test(tests/test_navigation_obstacle_avoidance.py TIMEOUT 240)
  # Controller bring-up + move_group startup + planning/execution is slower than the simpler
  # mapping-only tests -- default ctest TIMEOUT (60s) isn't enough.
  add_ros_isolated_launch_test(tests/test_moveit_smoke.py TIMEOUT 180)
endif()

ament_package()
