# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MAGNI POC is a **ROS 2 Jazzy** workspace implementing custom controllers, SLAM, navigation, and image recognition for the Ubiquity Robotics MAGNI platform. The same codebase targets three runtime contexts:

- **Simulation** — MAGNI spawned in Webots (`break_room.wbt`) with mock hardware, for development on a workstation.
- **Real robot** (`magni1`, `magni2`) — arm64 Raspberry Pi driving real motors (serial/i2c) and a YDLidar.
- **Control station** — an operator/visualization container (RViz) that talks to the robots over the network.

The workspace lives at `/home/ws` inside the dev/runtime containers; this path is hard-coded in scripts, the Dockerfiles, and `devcontainer.json` (`workspaceFolder`).

## Git Submodules

Three `src/` packages are git submodules and are **not present after a plain clone** — initialize them before building:

```bash
git submodule update --init --recursive
```

- `src/ubiquity_motor_ros2` — motor hardware interface + `ubiquity_motor_ros2_msgs`; provides the `ros2_control` `SystemInterface` plugin, `motors.launch.py`, and controller config `cfg/conf.yaml`.
- `src/YDLidar-SDK` — C++ SDK for the lidar (built with plain CMake, installs to `/usr/local`, **before** colcon).
- `src/ydlidar_ros2_driver` — ROS 2 driver wrapping the SDK (tracks the `humble` branch).

## Build

Builds happen inside the devcontainer (or a ROS Jazzy environment). Always source ROS first:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

- `./scripts/build-simulation.sh` — builds the simulation-relevant packages via `colcon build --base-paths ...` (motor msgs/interface first, then description/bringup/mapping/webots/control_station). Build order matters: `ubiquity_motor_ros2_msgs` and the YDLidar SDK must build before their dependents.
- The arm64 robot image (`Dockerfile`) builds the YDLidar SDK with raw `cmake/make/make install`, then runs colcon in two passes (motor packages, then the rest). Mirror this ordering if you change the build graph.
- `--symlink-install` is the convention so Python launch/config edits take effect without rebuilding.

## Run

```bash
# Simulation (Webots). Optional flags toggle conditional subsystems.
./scripts/launch-webots-simulation.sh [--rviz] [--mapping]

# Keyboard teleop (publishes stamped Twist on /cmd_vel)
./scripts/run-teleop-keyboard.sh

# Joystick teleop (PS3 mapping)
./scripts/launch-teleop-joy.sh

# Standalone mapping after Webots is up
ros2 launch magni_mapping mapping.launch.py use_sim_time:=true
```

On the real robot, `run-magni-container.sh <image>` runs the deployed image with the device passthrough the hardware needs (`/dev/serial0`→`/dev/ttyAMA0`, `/dev/i2c-1`, `/dev/ttyUSB0`, `/dev/input`, realtime ulimits). `run-control-station-container.sh <image>` runs the control-station image with X11/host networking.

## Test

```bash
# Run a package's tests (lint + unit)
colcon test --packages-select <pkg> && colcon test-result --verbose

# Integration / mapping smoke test
./scripts/run-integration-test.sh   # colcon test on magni_integration_tests
```

- Python packages (`magni_bringup`, `magni_control_station`) ship the standard ament lint tests: `test_flake8.py`, `test_pep257.py`, `test_copyright.py` under `test/`.
- `magni_mapping` has a config-validation test (`test/test_cartographer_config.py`); `magni_integration_tests` has launch-based tests (`tests/test_mapping_launch.py`) that expect a `drive.bag` rosbag at `src/magni_integration_tests/resources/drive.bag`.

## Architecture

### Two distinct bringup entry points

These are separate launch graphs — do not confuse them:

- **`magni_webots/launch/magni_sim_bringup.launch.py`** (driven by `launch-webots-simulation.sh`) — the simulation top level. Args: `rviz` and `mapping` (both default `false`). It includes `magni_spawn.launch.py` and conditionally `magni_mapping` + RViz.
- **`magni_bringup/launch/magni_bringup.launch.py`** — the **real-robot** top level (also the default `CMD` in the arm64 `Dockerfile`). Args: `enable_lidar` and `teleop` (both default `true`). It includes `ubiquity_motor_ros2/motors.launch.py`, the YDLidar driver (params overridden by `magni_bringup/config/ydlidar.yaml`), and joystick teleop.

### Webots spawn flow (`magni_spawn.launch.py`)

Node startup is **event-ordered**, and the sequencing is timing-sensitive:

1. `WebotsLauncher` starts Webots + `Ros2Supervisor`.
2. On supervisor exit/ready, `URDFSpawner` injects the MAGNI URDF (processed from `magni_description` xacro with `use_mock_hardware:=True`).
3. A `WebotsController` (`magni_driver`) loads `ubiquity_motor_ros2/cfg/conf.yaml` and **remaps** `/ubiquity_velocity_controller/cmd_vel`→`/cmd_vel` and `/ubiquity_velocity_controller/odom`→`/odom`.
4. `controller_manager` spawners bring up `ubiquity_velocity_controller` (diff-drive), then `joint_state_broadcaster` (gated by an `OnProcessExit` handler).
5. A **supervisor `NodeRemover`** (C++ Webots plugin, `src/magni_webots/src/NodeRemover.cpp`, registered in `magni_webots.xml`) is spawned via `/Ros2Supervisor/spawn_node_from_string` to strip unsupported caster joints from the simulated model.

### ros2_control

Motor control uses the standard ros2_control stack: a `SystemInterface` hardware plugin in `ubiquity_motor_ros2` (mock in sim, serial on hardware), the `ubiquity_velocity_controller` (diff drive) and `joint_state_broadcaster`, configured in `ubiquity_motor_ros2/cfg/conf.yaml`. The robot model and its `ros2_control` tags live in `src/magni_description/urdf/*.xacro` (`magni.control.xacro`).

### Mapping

Cartographer 2D fuses `/scan` (lidar) + `/odom`, configured in `src/magni_mapping/config/magni_2d.lua`. Cartographer provides the `map` frame and the `map->odom` TF. Frame conventions across the stack: `base_link`, `odom`, `map`. Use `use_sim_time:=true` everywhere in simulation.

### Standard topics

`cmd_vel`, `odom`, `scan`, `joint_states`, camera `image_raw` (compressed republishing via `scripts/run-image-transport-republish.sh`).

## CI / Deployment

All workflows in `.github/workflows/` are **manual (`workflow_dispatch`)**:

- `build.yaml` → builds `Dockerfile` for **linux/arm64**, pushes `ghcr.io/<owner>/magnipoc:<branch>`.
- `build-control-station.yml` → builds `Dockerfile.control_station`, pushes `...-control-station:<branch>`.
- `deploy.yaml` → runs Ansible (self-hosted runner) to deploy chosen target (`control_station`, `magni1`, `magni2`, `robots_only`, `everything`); the image tag is derived from the branch name.
- `mapping-smoke-test.yml` → builds a GPU sim image and runs the mapping integration test on a self-hosted GPU runner.

Deployment is Ansible-based: inventory in `ansible/inventories/hosts.ini`, playbooks in `ansible/playbooks/` (`deploy_magnipoc.yml`, `deploy_control_station.yml`). Images are pulled from GHCR; the deploy step tags by sanitized branch name.

## Conventions

- **Launch files** are Python; use `DeclareLaunchArgument` + `IfCondition(LaunchConfiguration(...))` to gate optional subsystems, and `IncludeLaunchDescription(PythonLaunchDescriptionSource(...))` to compose package launches.
- **Dependencies** are declared in each package's `package.xml` and resolved with `rosdep` (the devcontainer `postCreateCommand` runs `rosdep update && rosdep install`).
- **Webots plugins** extend the driver plugin interface and are registered in the package's `*.xml` and exported via `pluginlib` in `CMakeLists.txt`.
- **Architecture docs** are first-class: ADRs in `architecture/adr/`, design docs in `architecture/` (e.g. `cartographer_design.md`, `ydlidar_integration_design.md`), overview in `architecture/baseline_architecture.md`. Record significant design decisions as ADRs there.
- `build/`, `install/`, `log/` are gitignored; never commit them.

## Environment notes

- The devcontainer sets `ROS_DOMAIN_ID=42` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — match these when joining the DDS graph from another shell/host.
- Two devcontainer variants share one `Dockerfile`: `.devcontainer/cpu-simulation` and `.devcontainer/gpu-simulation` (adds `--gpus all` + NVIDIA PRIME offload env). Use GPU for Webots rendering performance.
- Inside the container, the `sws` alias re-sources the workspace (`source /ros_entrypoint.sh`).
