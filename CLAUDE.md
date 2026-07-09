# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

MAGNI POC is a ROS 2 (Jazzy) workspace implementing custom controllers for the Ubiquity Robotics MAGNI mobile
platform, simulated in Webots. It layers on: an optional arm (Seeed reBot Arm or Franka Research 3) and humanoid
torso mounted on the base, Cartographer 2D SLAM (LIDAR + odometry), and a real-hardware YDLidar driver as an
alternative to the simulated LIDAR. The long-term goal is a platform capable of navigation, SLAM, and image
recognition, deployed both to a control station and to physical robots via Ansible.

## Environment

- This repo is meant to be opened in the devcontainer (`workspaceFolder` is `/home/ws`). Two variants exist:
  `.devcontainer/cpu-simulation` and `.devcontainer/gpu-simulation` — pick GPU if the host has an NVIDIA GPU +
  Container Toolkit. `postCreateCommand` runs `rosdep update`/install automatically.
- ROS 2 distro is **jazzy** — always `source /opt/ros/jazzy/setup.bash` before building or running anything.
- Submodules live under `src/`: `ubiquity_motor_ros2`, `YDLidar-SDK`, `ydlidar_ros2_driver` (branch `humble`),
  `franka_description`, `rebotarm_ros2`. Run `git submodule update --init --recursive` after cloning/pulling if
  these packages look empty.

## Common commands

```bash
# Build everything
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Build only the simulation-relevant packages (matches scripts/build-simulation.sh)
./scripts/build-simulation.sh

# Launch the Webots simulation (arm/torso options below)
./scripts/launch-webots-simulation.sh [--rviz] [--mapping] [--no-arm] [--arm-family=rebotarm|franka] \
    [--robot-type=fr3] [--no-hand] [--no-torso] [--torso-left=tray|none] [--torso-head=screen|sensor_head|none]

# Teleop (in a second terminal, after the simulation is up)
./scripts/run-teleop-keyboard.sh
./scripts/launch-teleop-joy.sh

# Integration tests (magni_integration_tests package only)
./scripts/run-integration-test.sh
# equivalent to: colcon test --ctest-args tests --packages-select magni_integration_tests

# Single package test (standard colcon pattern)
colcon test --packages-select <package_name>
colcon test-result --verbose
```

There is no top-level lint/format command; each Python package carries its own `ament_lint`-based
`test_copyright.py` / `test_flake8.py` / `test_pep257.py`, run via `colcon test`.

## Architecture

### Package layout (`src/`)

- **magni_description** — URDF/xacro for the robot: base (`magni.urdf.xacro`), `ros2_control` config
  (`magni.control.xacro`), optional humanoid torso (`torso.urdf.xacro`) or tower mount, and per-arm-family control
  xacros (`rebotarm.control.xacro`, `franka_arm.control.xacro`). Sensor mounts (camera, sonar, LIDAR) are
  positioned via YAML "extrinsics" files under `urdf/extrinsics/` so a sensor's pose can change without editing
  xacro.
- **magni_webots** — Webots plugins/launch. `magni_spawn.launch.py` builds the robot description, spawns it via
  `URDFSpawner`, brings up `controller_manager` controllers, and spawns a `NodeRemover` supervisor plugin
  (`src/NodeRemover.cpp`) to strip nodes Webots doesn't need (ported from Ekumen's andino_webots). Per-arm
  controller YAMLs (`resource/rebotarm_controllers.yaml`, `resource/franka_arm_controllers.yaml`) are deep-merged
  with `ubiquity_motor_ros2/cfg/conf.yaml` at launch time into one temp file, because `WebotsController` only
  accepts a single `--params-file`.
- **ubiquity_motor_ros2** (submodule) — C++ hardware interface (`MotorHardware`) for the real motor controller,
  integrates with `ros2_control`/`controller_manager`; ships `ubiquity_motor_ros2_msgs` for custom motor state
  messages.
- **magni_bringup** — top-level launch glue: includes `magni_webots` spawn launch, conditionally includes
  `magni_mapping`, conditionally launches RViz. `rviz` and `mapping` are launch arguments.
- **magni_mapping** — Cartographer 2D config/launch (`magni_2d.lua` parameters), consumes `/scan` + odometry.
- **magni_control_station** — operator-side launch (RViz, teleop) meant to run on a separate control-station
  machine/container, not on the robot.
- **magni_teleop**, **video_recorder** — small C++ utility nodes (teleop passthrough, camera recording via
  `image_transport`).
- **magni_integration_tests** — `launch_testing`-based end-to-end tests (e.g. `test_mapping_launch.py` plays a
  recorded bag and drives Cartographer through a `WriteState` service call).
- **YDLidar-SDK**, **ydlidar_ros2_driver** (submodules) — real-hardware LIDAR support, alternative to Webots'
  simulated LIDAR; publishes `/scan` with `frame_id="laser"` to match the simulated setup (see
  `architecture/adr/003-integrate-ydlidar-real-lidar.md`).
- **franka_description**, **rebotarm_ros2** (submodules) — upstream arm description/driver packages consumed by
  `magni_description`'s arm xacros.

### Launch chain and data flow

```
magni_bringup.launch.py
  -> magni_webots/launch/magni_spawn.launch.py (Webots launcher + spawn-on-supervisor-exit)
       -> xacro-processes magni.urdf.xacro (env-var driven, see below) -> URDFSpawner spawns "magni"
       -> WebotsController ("magni_driver") loads merged controller params
       -> controller_manager spawns, in order: ubiquity_velocity_controller -> joint_state_broadcaster
          -> arm_controller -> (rebotarm only) gripper_controller
       -> NodeRemover supervisor plugin spawned via Ros2Supervisor service call
  -> magni_mapping/launch/mapping.launch.py (only if mapping:=true)
  -> rviz2 (only if rviz:=true)
```

`cmd_vel` -> `ubiquity_velocity_controller` -> `MotorHardware` (or Webots mock) -> wheel commands; `MotorHardware`
publishes `odom`, `JointState`, `MotorState`, diagnostics, battery. LIDAR/camera publish via `sensor_msgs`/
`image_transport`; Cartographer consumes `/scan` + `/odom` to produce `/map` when mapping is enabled.

### Arm/torso configuration quirk

`magni_spawn.launch.py` processes the robot's xacro *before* any `LaunchContext` exists, so arm/torso options
cannot be passed as `ros2 launch` arguments at that point — they are read from environment variables instead
(`MAGNI_ARM_INSTALLED`, `MAGNI_ARM_FAMILY`, `MAGNI_ARM_ROBOT_TYPE`, `MAGNI_ARM_HAND`, `MAGNI_TORSO_INSTALLED`,
`MAGNI_TORSO_LEFT`, `MAGNI_TORSO_HEAD`). `scripts/launch-webots-simulation.sh` sets these from its CLI flags —
if you add a new bringup entry point, either export the same variables first or extend this env-var pattern
rather than trying to thread a `LaunchConfiguration` through. Only one arm family is ever mounted at a time
(the URDF and controller YAML for the other family simply don't exist in that run).

### Architecture documentation & decisions

- `architecture/baseline_architecture.md` — component/data-flow overview, kept short by design.
- `architecture/adr/` — ADRs for major decisions (e.g. 002: Cartographer 2D choice, 003: YDLidar real-hardware
  integration). Add new ADRs here when making an architecturally significant decision.
- `architecture/cartographer_design.md`, `architecture/ydlidar_integration_design.md` — subsystem design docs.
- `.github/agents/ROS2Architect.agent.md` and `ROS2Developer.agent.md` define a two-agent workflow (Architect
  produces ADRs/plans and hands off to Developer for implementation) used by GitHub Copilot in this repo; mirror
  that separation of concerns (design artifact first, then code) for non-trivial architectural changes.

## CI/CD

- `.github/workflows/build.yaml` — manual (`workflow_dispatch`) build/push of the robot Docker image (`Dockerfile`)
  to `ghcr.io`, `linux/arm64`.
- `.github/workflows/build-control-station.yml` — builds `Dockerfile.control_station` (RViz/teleop/Nav2/Cartographer
  runtime, built from `src/magni_description`, `magni_teleop`, `magni_mapping`, `magni_control_station` only).
- `.github/workflows/deploy.yaml` — manual Ansible deploy to `control_station`, `magni1`, `magni2`, `robots_only`,
  or `everything`, via playbooks in `ansible/playbooks/`.
- `.github/workflows/mapping-smoke-test.yml` — self-hosted GPU runner; builds the GPU devcontainer image and runs
  the mapping integration test inside it.
