# MAGNI POC — Baseline Architecture

## Purpose ✅
A small, focused summary to quickly onboard developers and operators: what the system contains, how components interact, key dependencies, and how to run a smoke test.

---

## Scope & assumptions
- Focus: robot control simulation and hardware integration (magni in Webots + motor hardware interface).
- Assumes ROS 2 (jazzy) and Webots available (devcontainer contains packages and scripts).

---

## Components (short) 🔧
- **magni_description** — URDF/xacro for MAGNI (includes `ros2_control` configuration).
- **magni_webots** — Webots plugins and launch; spawns robot in `break_room.wbt`, includes `NodeRemover` plugin and bridges to simulation (uses `webots_ros2_driver`).
- **ubiquity_motor_ros2** — Motor hardware interface and node (C++), integrates with `ros2_control` and `controller_manager`; provides `MotorHardware`, `motor_node`, and a plugin description.
- **ubiquity_motor_ros2_msgs** — Custom message types for motor state.
- **magni_bringup** — Python/launch glue for starting bringup components.
- **video_recorder** — Utility node to capture camera topics via `image_transport`.

---

## Key external/system dependencies ⚙️
- ROS 2: jazzy (container/Dockerfiles reference `ros-jazzy-*`).
- Webots + `webots_ros2_driver` (simulation environment & plugins).
- ros2_control / controller_manager / hardware_interface (motor HW & controllers).
- Plugins & support libs: `pluginlib`, `image_transport`, `xacro`, `joint_state_publisher`.
- Common message packages: `sensor_msgs`, `geometry_msgs`, `std_msgs`, `diagnostic_msgs`.
- Build tools: `ament_cmake`, `ament_python`, `colcon`.

---

## Runtime topology & data flow (concise) 🔁
1. Launch (example): `./scripts/launch-webots-simulation.sh` → runs `magni_webots` launch.
2. Webots spawns MAGNI URDF (via `URDFSpawner`) and starts `Ros2Supervisor`.
3. `controller_manager` spawns `ubiquity_velocity_controller` and `joint_state_broadcaster` (config: `ubiquity_motor_ros2/cfg/conf.yaml`).
4. `cmd_vel` → `ubiquity_velocity_controller` → controller writes wheel commands to `MotorHardware` plugin.
5. `MotorHardware` interacts with physical serial interface (or mock in simulation) and publishes `MotorState`, `JointState`, `odom`, diagnostics, and battery state.
6. Cameras & LIDAR run in Webots and publish via `sensor_msgs` / `image_transport`; `video_recorder` can record camera topics.

---

## Important files & locations 📁
- Launch & scripts: `scripts/launch-webots-simulation.sh`, `src/magni_webots/launch/magni_spawn.launch.py`, `src/magni_bringup/launch/`.
- Robot model & control: `src/magni_description/urdf/*.xacro` (including `magni.control.xacro`).
- Motor HW & controllers: `src/ubiquity_motor_ros2/` (headers, `motor_hardware`, `motor_node`, `cfg/conf.yaml`).
- Plugins: `src/magni_webots/magni_webots.xml`, `src/magni_webots/src/NodeRemover.cpp`.

---

## Build & quick smoke test 🧪
1. In devcontainer (or host with required packages):
   - `source /opt/ros/jazzy/setup.bash`
   - `colcon build --symlink-install`
   - `source install/setup.bash`
2. Run simulation: `./scripts/launch-webots-simulation.sh`
3. In another terminal: `./scripts/run-teleop-keyboard.sh` → verify robot moves in Webots and topics `/cmd_vel`, `/odom`, `/joint_states`, and camera topics are present.
4. Observe controller and motor topics: `ros2 topic echo /motor_state` or `ros2 topic hz /camera/image_raw`.

---

## Risks & short notes ⚠️
- Target ROS 2 distro is `jazzy` (very new); portability to other distros may require dependency adjustments.
- Serial hardware access (`/dev/ttyAMA0`) needs platform-specific permissions when running on real robot.
- Webots supervisor/spawn requires correct node ordering; spawner events are sensitive to timing.

---

## Suggested next steps (short) ➕
- Add a minimal architecture diagram (SVG/ASCII) to `architecture/` (if desired).
- Add **Cartographer 2D mapping** (LIDAR + odometry) — design completed and saved to `architecture/cartographer_design.md`; **`magni_mapping` package created** with template configs, launch, and smoke-test script; next: integrate mapping into `magni_bringup` and wire CI smoke-test runner.
- Add **Nav2 navigation** (SLAM mode, reusing Cartographer's `/map` + TF, no AMCL) — design in `architecture/adr/004-nav2-navigation.md`; **`magni_navigation` package created** with `nav2_params.yaml` + `navigation.launch.py`, wired into `magni_sim_bringup.launch.py` via a `navigation` arg and `scripts/launch-webots-simulation.sh --nav2`; next: tune `controller_server`/costmap parameters against the real chassis footprint and consider AMCL + a saved map for deployment once a map is captured.
- Add a small integration test: launch simulation + publish `cmd_vel` programmatically and assert odom changes.
- Add README section summarizing the above quick checks, mapping steps, and common troubleshooting tips.

---

If you'd like, I can (a) add a minimal diagram, (b) implement the Cartographer configs + mapping launch (create `magni_mapping` package or add to `magni_bringup`), or (c) add the smoke-test launch and CI job. Which would you prefer next?